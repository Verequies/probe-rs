//! Implement chip-specific dual-core reset for RP235x

use crate::MemoryMappedRegister;
use crate::architecture::arm::armv6m::{Aircr, Demcr};
use crate::architecture::arm::dp::{Abort, Ctrl, DpAddress, DpRegister};
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{ArmDebugSequence, cortex_m_wait_for_reset};
use crate::architecture::arm::{ApV2Address, ArmError, FullyQualifiedApAddress};
use std::sync::Arc;
use std::time::{Duration, Instant};

const SIO_CPUID_OFFSET: u64 = 0xd000_0000;
const RP_AP: FullyQualifiedApAddress =
    FullyQualifiedApAddress::v2_with_dp(DpAddress::Default, ApV2Address(Some(0x80000)));

/// An address in RAM, used for validating that the core is working.
const RAM_ADDRESS: u64 = 0x2000_0000;

/// Speed to use during the rescue reset sequence. The RP2350's debug
/// subsystem needs time to reinitialize after RESCUE_RESTART; running
/// at a low speed naturally spaces out the transactions enough that we
/// don't race the chip's internal reset sequencer.
const RESET_SPEED_KHZ: u32 = 1_000;

/// Resetting the core can sometimes take multiple attempts. Abandon reset
/// if it takes longer than this duration. During testing of over 5000 reset-
/// program cycles, the longest observed reset cycle was 465 ms after 114
/// reset attempts. It may be that this is attempting to disable XIP on the SPI
/// flash, but it is ultimately unclear what's causing the delay.
const RESET_TIMEOUT: Duration = Duration::from_secs(1);

/// Debug implementation for RP235x
#[derive(Debug)]
pub struct Rp235x {}

impl Rp235x {
    /// Create a debug sequencer for a Raspberry Pi RP235x
    pub fn create() -> Arc<Self> {
        Arc::new(Rp235x {})
    }

    /// Clear any pending FAULT/STICKYERR on the DP so subsequent
    /// register reads don't fail. Required when retrying after a FAULT —
    /// the DAP refuses all further transactions until the sticky error
    /// is cleared via the ABORT register.
    fn clear_dp_errors(
        &self,
        arm_interface: &mut dyn crate::architecture::arm::ArmDebugInterface,
        dp: DpAddress,
    ) {
        let mut abort = Abort(0);
        abort.set_stkcmpclr(true);
        abort.set_stkerrclr(true);
        abort.set_wderrclr(true);
        abort.set_orunerrclr(true);
        let _ = arm_interface.write_raw_dp_register(dp, Abort::ADDRESS, abort.into());
    }

    /// Perform an AIRCR SYSRESETREQ reset.
    ///
    /// Returns `Ok(())` if the reset succeeded and RAM is stable afterwards.
    /// Returns `Err` if the reset fails or RAM is unstable (e.g. core 1 is
    /// running and corrupting memory), in which case the caller should fall
    /// back to [`perform_rescue_reset`].
    fn perform_aircr_reset(
        &self,
        core: &mut dyn ArmMemoryInterface,
        core_type: crate::CoreType,
        debug_base: Option<u64>,
        should_catch_reset: bool,
    ) -> Result<(), ArmError> {
        if should_catch_reset {
            self.reset_catch_set(core, core_type, debug_base)?;
        }

        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);
        core.write_word_32(Aircr::get_mmio_address(), aircr.into())?;

        cortex_m_wait_for_reset(core)?;

        // Verify that RAM is stable after reset. If core 1 is running and
        // corrupting memory the readback will not match, indicating that a
        // rescue reset is required to bring the chip to a known state.
        core.write_word_32(RAM_ADDRESS, 0xDEAD_BEEF)?;
        let readback = core.read_word_32(RAM_ADDRESS)?;
        if readback != 0xDEAD_BEEF {
            return Err(ArmError::Other(
                "RAM verification failed after AIRCR reset, core 1 may be corrupting memory".into(),
            ));
        }

        Ok(())
    }

    /// Perform a full rescue reset sequence.
    ///
    /// This is used when an AIRCR reset is insufficient — for example
    /// when core 1 is running and corrupting RAM. The rescue reset halts the
    /// entire SoC via the RP-AP before restarting it, ensuring both cores
    /// are in a known state before flashing begins.
    ///
    /// See: 'RP2350 Datasheet', sections:
    ///   3.5.8: Rescue Reset
    ///   3.5.10.1: RP-AP list of registers
    fn perform_rescue_reset(
        &self,
        core: &mut dyn ArmMemoryInterface,
        core_type: crate::CoreType,
        debug_base: Option<u64>,
        should_catch_reset: bool,
    ) -> Result<(), ArmError> {
        let ap = core.fully_qualified_address();
        let arm_interface = core.get_arm_debug_interface()?;

        // Take note of the existing values for CTRL. We will need to restore
        // these after entering rescue mode.
        let existing_core_0 = arm_interface.read_raw_dp_register(ap.dp(), Ctrl::ADDRESS)?;
        tracing::trace!("Core 0 DP_CTRL: {existing_core_0:08x}");

        // Drop to a safe speed for the rescue reset sequence. At high speeds
        // the AP transactions after RESCUE_RESTART arrive before the RP2350's
        // debug subsystem has reinitialized, causing FAULT on DRW. Running at
        // a low speed naturally spaces transactions far enough apart that the
        // chip always has time to recover.
        let mut probe_speed: Option<u32> = None;
        if let Some(probe) = arm_interface.try_dap_probe_mut() {
            probe_speed = Some(probe.speed_khz());
            tracing::debug!("Lowering SWD speed to {RESET_SPEED_KHZ} kHz for reset sequence");
            let _ = probe.set_speed(RESET_SPEED_KHZ);
        }

        // CTRL register offset within RP-AP.
        const CTRL: u64 = 0;
        const RESCUE_RESTART: u32 = 0x8000_0000;

        // Poke 1 then 0 to CTRL.RESCUE_RESTART to trigger rescue reset. This
        // causes a flag to be set in POWMAN CHIP_RESET (RESCUE) and the SoC
        // to reset. The BootROM checks for the RESCUE flag and, if set, halts
        // the system so we can attach as usual.
        let ctrl = arm_interface.read_raw_ap_register(&RP_AP, CTRL)?;
        arm_interface.write_raw_ap_register(&RP_AP, CTRL, ctrl | RESCUE_RESTART)?;
        let ctrl = arm_interface.read_raw_ap_register(&RP_AP, CTRL)?;
        arm_interface.write_raw_ap_register(&RP_AP, CTRL, ctrl & !RESCUE_RESTART)?;

        // Clear any FAULT from the rescue poke before proceeding.
        self.clear_dp_errors(arm_interface, ap.dp());

        let new_core_0 = arm_interface.read_raw_dp_register(ap.dp(), Ctrl::ADDRESS)?;
        tracing::trace!("new core 0 CTRL: {new_core_0:08x}");

        // Start the debug core back up which brings it out of Rescue Mode.
        self.debug_core_start(arm_interface, &ap, core_type, debug_base, None)?;

        // Restore the ctrl values.
        tracing::trace!("Restoring ctrl values");
        arm_interface.write_raw_dp_register(ap.dp(), Ctrl::ADDRESS, existing_core_0)?;

        // Restore speed before handing back to core-level operations.
        if let (Some(probe), Some(speed)) = (arm_interface.try_dap_probe_mut(), probe_speed) {
            tracing::debug!("Restoring SWD speed to {speed} kHz");
            let _ = probe.set_speed(speed);
        }

        // Finish with an AIRCR reset to bring the chip fully out of rescue mode.
        self.perform_aircr_reset(core, core_type, debug_base, should_catch_reset)
    }
}

impl ArmDebugSequence for Rp235x {
    fn reset_system(
        &self,
        core: &mut dyn ArmMemoryInterface,
        core_type: crate::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        tracing::trace!("reset_system(interface, {core_type:?}, {debug_base:x?})");

        // Only perform a system reset from core 0.
        let core_id = core.read_word_32(SIO_CPUID_OFFSET)?;
        if core_id != 0 {
            tracing::warn!("Skipping reset of core {core_id}");
            return Ok(());
        }

        // Since we're resetting the core, the catch_reset flag will get lost.
        // Note whether we should re-set it after the reset completes.
        let should_catch_reset =
            Demcr(core.read_word_32(Demcr::get_mmio_address())?).vc_corereset();

        // Attempt an AIRCR reset first. If RAM is stable afterwards,
        // the chip is in a good state and we can proceed without a rescue
        // reset. This is the common case during iterative development.
        if self
            .perform_aircr_reset(core, core_type, debug_base, should_catch_reset)
            .is_ok()
        {
            tracing::info!("AIRCR reset succeeded, skipping rescue reset");
            return Ok(());
        }

        // RAM was unstable after AIRCR reset, indicating core 1 is running
        // and corrupting memory. Fall back to the rescue reset sequence which
        // halts both cores via the RP-AP before restarting.
        //
        // Reset seems to get stuck in a state where RAM is inaccessible. A
        // second reset fixes this about 20% of the time. Perform multiple
        // attempts to get the board into a functioning state. Note that a
        // full rescue reset is required in this case -- simply resetting the
        // core does not get it into a functioning state.
        tracing::info!("AIRCR reset failed or RAM unstable, falling back to rescue reset");

        let start = Instant::now();
        let mut attempt = 0;
        loop {
            attempt += 1;
            tracing::debug!("Performing rescue reset (attempt {attempt})...");

            let Err(e) = self.perform_rescue_reset(core, core_type, debug_base, should_catch_reset)
            else {
                tracing::info!(
                    "Finished RP235x rescue reset after {attempt} attempts and {} ms",
                    start.elapsed().as_millis()
                );
                return Ok(());
            };

            if start.elapsed() > RESET_TIMEOUT {
                tracing::error!(
                    "Rescue reset failed after {attempt} attempts and {} ms: {e}",
                    start.elapsed().as_millis()
                );
                return Err(e);
            }

            tracing::debug!("Attempt {attempt} failed ({e}), retrying");

            // Clear sticky DP errors before next attempt.
            let dp = core.fully_qualified_address().dp();
            if let Ok(arm_interface) = core.get_arm_debug_interface() {
                self.clear_dp_errors(arm_interface, dp);
            }
        }
    }
}
