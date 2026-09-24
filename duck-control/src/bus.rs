//! The Dynamixel bus through an OpenRB-150 USB bridge.
//!
//! One servo `sync_read`, one body-IMU I²C poll, and one servo `sync_write` per tick. The
//! LSM6DSV16X shares the Radxa's Qwiic bus with the head sensors, not the Dynamixel wire.
//! The OpenRB runs ROBOTIS's factory `usb_to_dynamixel` sketch: it forwards the host's raw
//! Protocol 2 packets rather than owning the control loop, so servo configuration, sensing,
//! safety, and policy execution remain here on the Linux host.
//!
//! Every sync read here is a **fast** sync read (protocol 2.0 instruction 0x8A): the devices
//! append their answers to one status packet from the broadcast id instead of each sending
//! its own, which drops fifteen packet headers and fifteen bus turnarounds from the tick.
//! `bus.fast_sync_read` in `robotd.toml` turns it off for a robot whose devices do not
//! implement the instruction. See [`open_controller`].
//!
//! Battery and thermals are the one thing that does not fit that shape: they live at registers
//! outside the block the tick fetches, so [`RobotIo::slow_sensors`] is a transaction of its
//! own, meant to be called about once a second rather than every tick.
//!
//! Written against `rustypot`, but the *numbers* — conversion factors and the EEPROM
//! registers asserted at startup — come from `microduck_runtime`, where they were arrived
//! at against real hardware. See [`crate::model`].

use std::error::Error;
use std::f64::consts::PI;
use std::io;
use std::path::Path;
use std::time::Duration;

use rustypot::servo::dynamixel::xl330::Xl330Controller;

use crate::imu::{ImuData, SflpDecoder};
use crate::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{
    BAUD_RATE, EXPECTED_REGISTERS, FACTORY_BAUD_RATE, FACTORY_ID, JOINT_IDS, JOINT_NAMES,
    NUM_JOINTS,
};

/// Start of the contiguous block read every tick: `present_pwm`, `present_current`,
/// `present_velocity`, `present_position`. Twelve bytes covers all four.
const READ_ADDR: u8 = 124;
const READ_LEN: u8 = 12;

/// 0.229 rev/min per count, in rad/s.
const RAD_PER_SEC_PER_COUNT: f64 = 0.229 * (2.0 * PI / 60.0);

/// The second, slower block: `present_input_voltage` (144, `u16`) then `present_temperature`
/// (146, `u8`). Three bytes covers both, so voltage and thermals cost *one* extra transaction
/// between them rather than two — see [`RobotIo::slow_sensors`].
///
/// It starts eight bytes past the end of the tick's block, which is why it cannot simply be
/// folded into [`READ_LEN`]: the gap is `velocity_trajectory`/`position_trajectory`, twelve
/// bytes nothing here wants, and a servo answering a 22-byte read every tick would cost more
/// bus time than the two transactions do.
const SLOW_READ_ADDR: u8 = 144;
const SLOW_READ_LEN: u8 = 3;

/// `present_input_voltage` counts 0.1 V each.
const VOLTS_PER_COUNT: f64 = 0.1;

/// A healthy 15-device read completes well inside this. Capping it means a missing device
/// costs a bounded hiccup rather than stalling the loop on the serial driver's default.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

/// Let the OpenRB factory bridge apply USB CDC line coding before sending a packet.
///
/// Its sketch forwards available USB bytes *before* checking whether `USB.baud()` changed. A
/// packet written immediately after open could therefore leave the DXL UART at the previous
/// physical baud. This short, one-time pause gives the otherwise tight firmware loop a pass with
/// no payload first. It applies to normal startup, USB recovery, and the 1 Mbaud/57,600 baud
/// switches used to adopt a factory-fresh servo; it is never paid per control tick.
const OPENRB_LINE_CODING_SETTLE: Duration = Duration::from_millis(20);

/// How long a servo is off the bus after a REBOOT before it answers again — "a few hundred
/// milliseconds" per [`RobotIo::reboot`], with margin. Pinging too early would read a servo
/// that is merely still booting as one whose flash failed, and fail an adoption that worked.
const REBOOT_SETTLE: Duration = Duration::from_millis(500);

/// Pause after each EEPROM write. The servo acknowledges before the cell is necessarily
/// committed, and the writes here happen once per motor swap, so waiting costs nothing and
/// removes the one race the datasheet leaves open.
const EEPROM_SETTLE: Duration = Duration::from_millis(20);

/// Run of consecutive stale reads at which the journal says something.
///
/// The shipped sensor rate is above the control rate (60 Hz for the 50 Hz loop), so normal clock
/// phase can create one empty poll but not three in a row. Even at the allowed 1 kHz control-rate
/// ceiling, the 480 Hz SFLP ceiling produces at most two consecutive empty polls. On the third
/// miss the held orientation is about 60 ms old in the shipped configuration: stop treating it
/// as a complete policy sample and enter the loop's bounded coast path. Kept in step with
/// `ImuHealth::FROZEN_RUN`; the hardware layer deliberately does not depend on the IPC vocabulary,
/// so the number lives in both places.
const STALE_RUN_WARN: u64 = 3;

/// Detects an IMU poll that produced no new SFLP quaternion.
///
/// Split out from the read path so it can be tested without a serial port: the fault it
/// describes is one nothing else on the robot reports, and it would otherwise be verifiable
/// only against broken hardware.
#[derive(Debug, Default)]
struct StaleImuTracker {
    stale: ImuStale,
}

impl StaleImuTracker {
    /// Records whether a poll yielded a new sample and returns the current stale run.
    fn observe(&mut self, fresh: bool) -> u64 {
        if fresh {
            self.stale.run = 0;
        } else {
            self.stale.total = self.stale.total.saturating_add(1);
            self.stale.run = self.stale.run.saturating_add(1);
        }
        self.stale.run
    }

    fn frozen(&self) -> bool {
        self.stale.run >= STALE_RUN_WARN
    }
}

/// Recovery state for the host USB link while the terminal-powered OpenRB itself stays running.
///
/// Reopening is deliberately deferred until the next complete read. Reopening inside the
/// failing transaction would let the rest of that same coasted tick write an old target through
/// the new handle. After a reopen, motion-capable commands remain blocked until one complete
/// servo + body-IMU sample has landed; only then can the ordinary read-before-write tick resume.
/// Torque-off stays available once the replacement handle itself is live.
///
/// This is intentionally not MCU-reset recovery. Resetting/reflashing the OpenRB cycles its DXL
/// power FET, which resets servo RAM (including torque and gains). This layer never silently
/// re-enables either; after such a reset the operator explicitly relaxes and initializes the
/// robot so the high-level bring-up state and the servo RAM agree again.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum PortRecovery {
    #[default]
    Ready,
    ReopenBeforeRead,
    AwaitingCompleteRead,
}

impl PortRecovery {
    /// Remember a real host-port failure. A servo response timeout, checksum error, or wrong id
    /// is a bus/device miss and must not churn the USB device.
    fn observe(&mut self, error: &(dyn Error + 'static), port_present: bool) -> bool {
        if !should_recover_port(error, port_present) {
            return false;
        }
        let newly_armed = *self != Self::ReopenBeforeRead;
        *self = Self::ReopenBeforeRead;
        newly_armed
    }

    fn needs_reopen(self) -> bool {
        self == Self::ReopenBeforeRead
    }

    fn reopened(&mut self) {
        *self = Self::AwaitingCompleteRead;
    }

    fn complete_read(&mut self) {
        *self = Self::Ready;
    }

    fn may_command(self) -> bool {
        self == Self::Ready
    }

    /// Torque-off may bypass the complete-sample gate once a real controller handle has been
    /// reopened. It must not run in `ReopenBeforeRead`: [`DynamixelIo::reopen`] has already
    /// dropped the dead handle before an open failure returns, so no serial controller exists in
    /// that state.
    fn may_disable_torque(self) -> bool {
        self != Self::ReopenBeforeRead
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PortErrorClass {
    Host,
    Timeout,
    Other,
}

/// Classify a boxed `rustypot` error once, preserving host-device failures separately from a
/// servo that simply missed its response deadline.
fn classify_port_error(error: &(dyn Error + 'static)) -> PortErrorClass {
    let mut current = Some(error);
    let mut saw_timeout = false;
    while let Some(error) = current {
        if let Some(error) = error.downcast_ref::<io::Error>() {
            match error.kind() {
                io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock => saw_timeout = true,
                io::ErrorKind::Interrupted => {}
                _ => return PortErrorClass::Host,
            }
        } else if let Some(error) = error.downcast_ref::<serialport::Error>() {
            match error.kind() {
                serialport::ErrorKind::NoDevice => return PortErrorClass::Host,
                serialport::ErrorKind::Io(io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock) => {
                    saw_timeout = true
                }
                serialport::ErrorKind::Io(io::ErrorKind::Interrupted)
                | serialport::ErrorKind::InvalidInput
                | serialport::ErrorKind::Unknown => {}
                serialport::ErrorKind::Io(_) => return PortErrorClass::Host,
            }
        } else if matches!(
            error.downcast_ref::<rustypot::CommunicationErrorKind>(),
            Some(rustypot::CommunicationErrorKind::TimeoutError)
        ) {
            saw_timeout = true;
        }
        current = error.source();
    }
    if saw_timeout {
        PortErrorClass::Timeout
    } else {
        PortErrorClass::Other
    }
}

/// `rustypot` reduces a failed packet send to a timeout. The stable udev link supplies the missing
/// distinction: a timeout while the link resolves is a servo miss; one after it vanished reopens.
fn should_recover_port(error: &(dyn Error + 'static), port_present: bool) -> bool {
    match classify_port_error(error) {
        PortErrorClass::Host => true,
        PortErrorClass::Timeout => !port_present,
        PortErrorClass::Other => false,
    }
}

pub struct DynamixelIo {
    controller: Xl330Controller,
    /// Kept so the port can be reopened at the factory baud rate: `rustypot` owns the serial
    /// handle outright and offers no way to change its speed in place.
    port: String,
    /// Kept for the same reason as `port`: [`Self::reopen`] builds a new controller, and one
    /// built without this would silently drop back to a plain sync read for the rest of the
    /// process — a motor swap quietly halving the tick's bus budget.
    fast_sync_read: bool,
    /// Live USB disconnects are reopened through `port`, but no command is admitted until a
    /// complete read through that new handle has succeeded.
    port_recovery: PortRecovery,
    body_imu: qwiic_imu::Sensor,
    imu: SflpDecoder,
    last_imu: ImuData,
    /// Successful polls with no new SFLP quaternion. The control loop holds `last_imu`
    /// for those ticks, so freshness has to be counted separately.
    stale_imu: StaleImuTracker,
}

impl DynamixelIo {
    /// Open the bus. `fast_sync_read` is `bus.fast_sync_read` from `robotd.toml` — see
    /// [`open_controller`] for what it costs to have wrong.
    pub fn open(
        port: &str,
        fast_sync_read: bool,
        imu_bus: &Path,
        imu_address: u8,
        requested_hz: u16,
    ) -> Result<Self> {
        let controller = open_controller(port, BAUD_RATE, fast_sync_read)?;
        let body_imu = qwiic_imu::Sensor::open(imu_bus, imu_address, requested_hz)
            .map_err(|e| IoError::Bus(format!("open body IMU on {}: {e:#}", imu_bus.display())))?;

        Ok(Self {
            controller,
            port: port.to_owned(),
            fast_sync_read,
            port_recovery: PortRecovery::default(),
            body_imu,
            imu: SflpDecoder::default(),
            last_imu: ImuData::default(),
            stale_imu: StaleImuTracker::default(),
        })
    }

    /// Effective SFLP rate after rounding the control rate to a supported sensor rung.
    pub fn body_imu_rate_hz(&self) -> u16 {
        self.body_imu.rate_hz()
    }

    /// Turn a `rustypot` failure into this crate's error while retaining the one distinction a
    /// hot-pluggable USB bridge needs: host-port loss versus a servo/protocol miss.
    fn observe_port_error(&mut self, error: &(dyn Error + 'static)) {
        if self
            .port_recovery
            .observe(error, Path::new(&self.port).exists())
        {
            tracing::warn!(
                port = %self.port,
                "OpenRB USB link disappeared; will reopen it before the next servo read"
            );
        }
    }

    fn servo_error(&mut self, what: &str, error: Box<dyn Error>) -> IoError {
        self.observe_port_error(error.as_ref());
        IoError::Bus(format!("{what}: {error}"))
    }

    /// Reopen a re-enumerated OpenRB only at the start of a read-before-write tick.
    fn recover_port_before_read(&mut self) -> Result<()> {
        if !self.port_recovery.needs_reopen() {
            return Ok(());
        }
        self.reopen(BAUD_RATE)?;
        self.port_recovery.reopened();
        tracing::warn!(
            port = %self.port,
            "OpenRB USB link reopened; waiting for a complete sensor sample before commanding"
        );
        Ok(())
    }

    /// Motion-capable commands through a newly reopened bridge are refused until the tick has
    /// fresh joint and body-IMU data. Torque-off is deliberately exempt: losing the required IMU
    /// must never prevent an operator or shutdown path from relaxing otherwise reachable servos.
    /// This makes recovery follow the same read-before-write invariant as startup without
    /// turning a sensor failure into a torque-off interlock.
    fn require_complete_read(&self) -> Result<()> {
        if self.port_recovery.may_command() {
            Ok(())
        } else {
            Err(IoError::Bus(
                "OpenRB USB link is recovering; refusing a servo command until a complete sensor \
                 read succeeds"
                    .to_owned(),
            ))
        }
    }

    /// Assert — and correct — the EEPROM registers the control loop depends on.
    ///
    /// Returns how many needed fixing. A servo that has been factory-reset or swapped in
    /// arrives with `return_delay_time = 250`, which alone would eat 40% of the tick
    /// budget across the bus. Checking costs one read per register at startup and removes
    /// a whole class of "why is it slow on this robot".
    pub fn check_registers(&mut self) -> Result<usize> {
        let mut fixed = 0;
        for &id in &JOINT_IDS {
            fixed += self.check_registers_of(id)?;
        }
        Ok(fixed)
    }

    /// [`Self::check_registers`] for one servo.
    fn check_registers_of(&mut self, id: u8) -> Result<usize> {
        let mut fixed = 0;
        for &(name, want) in EXPECTED_REGISTERS {
            // rustypot returns a Vec even for a single-id read. An empty one means the
            // servo did not answer, which must not be read as "register is fine".
            let raw = match name {
                "return_delay_time" => self.controller.read_return_delay_time(id),
                "baud_rate" => self.controller.read_baud_rate(id),
                "pwm_slope" => self.controller.read_pwm_slope(id),
                "shutdown" => self.controller.read_shutdown(id),
                other => unreachable!("unhandled register {other}"),
            }
            .map_err(|e| IoError::Bus(format!("read {name} on {id}: {e}")))?;

            let got = *raw.first().ok_or(IoError::ShortRead {
                what: "register read",
                expected: 1,
                got: 0,
            })?;

            if got == want {
                continue;
            }
            tracing::warn!(id, register = name, got, want, "correcting motor register");
            match name {
                "return_delay_time" => self.controller.write_return_delay_time(id, want),
                "baud_rate" => self.controller.write_baud_rate(id, want),
                "pwm_slope" => self.controller.write_pwm_slope(id, want),
                "shutdown" => self.controller.write_shutdown(id, want),
                other => unreachable!("unhandled register {other}"),
            }
            .map_err(|e| IoError::Bus(format!("write {name} on {id}: {e}")))?;
            std::thread::sleep(EEPROM_SETTLE);
            fixed += 1;
        }
        Ok(fixed)
    }

    /// The expected servo IDs that do not answer a ping, in [`JOINT_IDS`] order.
    ///
    /// Fifteen pings, each bounded by [`READ_TIMEOUT`], so about half a second when the servos
    /// are unpowered and a few milliseconds when they are not. Run once at startup: this is
    /// what decides whether [`Self::adopt_replacement`] has anything to do, and it is the only
    /// bus traffic the replacement path costs a robot whose servos are all present.
    pub fn missing_servos(&mut self) -> Result<Vec<u8>> {
        let mut missing = Vec::new();
        for &id in &JOINT_IDS {
            let answered = self
                .controller
                .ping(id)
                .map_err(|e| IoError::Bus(format!("ping {id}: {e}")))?;
            if !answered {
                missing.push(id);
            }
        }
        Ok(missing)
    }

    /// Flash a factory-fresh servo so it takes the place of the one that is missing.
    ///
    /// A new XL330 answers as ID 1 at 57 600 baud. Neither is used on this bus, so when exactly
    /// one expected servo is silent the new one can be found, given the missing ID, switched to
    /// the bus's speed, and then handed the same EEPROM check every other servo gets. That is
    /// the whole of a motor swap: nobody has to run a configuration tool first.
    ///
    /// The servo is rebooted at the end, deliberately. A servo flashed this way comes out of it
    /// with its hardware-error alert set (the observation behind this whole path), and that
    /// alert holds torque off until the servo is power-cycled or rebooted. Rebooting here means
    /// the servo that comes out of this is indistinguishable from one that was always there.
    ///
    /// Returns `Ok(false)` when nothing answers at the factory defaults: the servo is simply
    /// missing, or was replaced by one that is not fresh. The bus is back at [`BAUD_RATE`]
    /// either way, so the caller can keep waiting on it.
    pub fn adopt_replacement(&mut self, id: u8) -> Result<bool> {
        let name = JOINT_IDS
            .iter()
            .position(|&j| j == id)
            .map(|i| JOINT_NAMES[i])
            .ok_or_else(|| IoError::Bus(format!("{id} is not a joint id")))?;

        // A servo that was already re-flashed to 1 Mbps but kept its ID is the one case where
        // reopening the port would lose it; look at this speed first.
        let baud = if self.ping_fresh()? {
            BAUD_RATE
        } else {
            self.reopen(FACTORY_BAUD_RATE)?;
            if !self.ping_fresh()? {
                self.reopen(BAUD_RATE)?;
                return Ok(false);
            }
            FACTORY_BAUD_RATE
        };
        tracing::warn!(
            id,
            joint = name,
            found_at_baud = baud,
            "factory-fresh servo on the bus; flashing it as the missing joint"
        );

        // ID first, then the baud rate: the servo answers the second write at the old speed
        // and switches only afterwards, so both are acknowledged. The other order would need
        // a reopen between the two writes for nothing.
        self.controller
            .write_id(FACTORY_ID, id)
            .map_err(|e| IoError::Bus(format!("write id {id} on {FACTORY_ID}: {e}")))?;
        std::thread::sleep(EEPROM_SETTLE);
        if baud != BAUD_RATE {
            let want = EXPECTED_REGISTERS
                .iter()
                .find(|(n, _)| *n == "baud_rate")
                .map(|&(_, v)| v)
                .expect("baud_rate is an expected register");
            self.controller
                .write_baud_rate(id, want)
                .map_err(|e| IoError::Bus(format!("write baud_rate on {id}: {e}")))?;
            std::thread::sleep(EEPROM_SETTLE);
            self.reopen(BAUD_RATE)?;
        }

        // Now an ordinary servo at the right address: the same check the others get pins
        // return_delay_time and the rest.
        let fixed = self.check_registers_of(id)?;

        RobotIo::reboot(self, id)?;
        std::thread::sleep(REBOOT_SETTLE);
        let back = self
            .controller
            .ping(id)
            .map_err(|e| IoError::Bus(format!("ping {id} after reboot: {e}")))?;
        if !back {
            return Err(IoError::Bus(format!(
                "servo {id} ({name}) was flashed but did not come back from its reboot"
            )));
        }
        // The reboot exists to clear this; say so if it did not, because a servo that keeps
        // its alert will hold torque off and the symptom — one limp joint — points nowhere.
        let hardware_error = self
            .controller
            .read_hardware_error_status(id)
            .map_err(|e| IoError::Bus(format!("read hardware_error_status on {id}: {e}")))?
            .first()
            .copied()
            .unwrap_or(0);
        if hardware_error != 0 {
            tracing::error!(
                id,
                joint = name,
                hardware_error,
                "replacement servo still reports a hardware error after its reboot"
            );
        }
        tracing::warn!(
            id,
            joint = name,
            registers_fixed = fixed,
            "replacement servo adopted"
        );
        Ok(true)
    }

    /// Does anything answer at the factory ID, at whatever speed the port is open at?
    fn ping_fresh(&mut self) -> Result<bool> {
        let result = self.controller.ping(FACTORY_ID);
        result.map_err(|e| self.servo_error(&format!("ping factory id {FACTORY_ID}"), e))
    }

    /// Close the port and open it again at `baud`.
    ///
    /// The old handle has to be gone first: `serialport` opens ttys exclusively, so opening a
    /// second handle while the first lives fails with `EBUSY`. Hence the placeholder controller
    /// — one with no port, never used — standing in while the real one is dropped.
    fn reopen(&mut self, baud: u32) -> Result<()> {
        self.controller = Xl330Controller::new();
        self.controller = open_controller(&self.port, baud, self.fast_sync_read)?;
        Ok(())
    }

    /// Present positions only — a lighter read than [`RobotIo::read`], used once at startup
    /// to adopt the pose the robot is already in.
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let result = self.controller.sync_read_present_position(&JOINT_IDS);
        let values = result.map_err(|e| self.servo_error("read present positions", e))?;
        if values.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "present positions",
                expected: NUM_JOINTS,
                got: values.len(),
            });
        }
        let mut out = [0.0; NUM_JOINTS];
        out.copy_from_slice(&values);
        Ok(out)
    }

    /// Torque on every servo.
    ///
    /// One transaction per joint, so this is not something to call per tick — the control loop calls
    /// it once, when someone enables the policy on a limp robot. See [`RobotIo::set_torque`] for what
    /// has *not* changed: nothing touches torque because a process started.
    ///
    /// **Every servo is written, whatever the others said.** Fourteen acknowledged transactions
    /// in a row, and one dropped ack used to end the loop there — which for `on = false` on the
    /// way to a power-off meant a robot that sat down and switched off with half its legs still
    /// locked, because the tick that saw the error was the last one that could have retried.
    /// Writing the rest costs the same as it would have, and the error names every joint that
    /// did not answer so the caller can decide whether to ask again.
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        // Energising joints is a motion-capable command and needs a fresh complete observation.
        // De-energising them is the fail-safe path and must remain available even when the body
        // IMU is what kept recovery from completing.
        if on {
            self.require_complete_read()?;
        } else if !self.port_recovery.may_disable_torque() {
            return Err(IoError::Bus(
                "OpenRB USB link is absent; retry torque-off after the port reopens".to_owned(),
            ));
        }
        let mut failed = Vec::new();
        for &id in &JOINT_IDS {
            if let Err(e) = self.controller.write_torque_enable(id, on) {
                self.observe_port_error(e.as_ref());
                failed.push(format!("torque {on} on {id}: {e}"));
            }
        }
        if failed.is_empty() {
            Ok(())
        } else {
            Err(IoError::Bus(failed.join("; ")))
        }
    }

    /// Ramp every joint from where it is now to `target`, linearly.
    ///
    /// Only ever called by an explicit `init` — the control loop must never move the robot
    /// on its own, because that would make an update restart a fall risk. Blocking, and
    /// deliberately so: nothing else should be talking to the bus while this runs.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }
}

/// The OpenRB factory USB bridge at `baud`, wrapped in a Protocol 2 controller.
///
/// `with_fast_sync_read` routes every `sync_read_*` through instruction 0x8A, so it covers
/// the tick's motor read, [`RobotIo::slow_sensors`] and [`DynamixelIo::present_positions`]
/// without any of them naming it. The saving is a packet header and a turnaround — the
/// device's `return_delay_time`, which [`EXPECTED_REGISTERS`] pins low precisely because it
/// is paid per device — for each of the fifteen servos on the bus.
///
/// It is all or nothing: one status packet carries every block, so a device whose firmware
/// does not implement 0x8A does not answer and the whole read times out. That is the same
/// shape of failure a silent servo already causes on a plain sync read, and the tick coasts
/// over a dropped read either way. XL330 firmware needs to be v46 or newer, which is a
/// property of a robot's hardware and the reason `fast_sync_read` is a setting at all rather
/// than something this code decides.
///
/// Nothing here probes for support. A device that does not answer looks exactly like one that
/// is unpowered, and a startup probe would have to tell those apart to say anything useful —
/// so the answer is a key someone turns off, not a guess this code makes every boot.
fn open_controller(port: &str, baud: u32, fast_sync_read: bool) -> Result<Xl330Controller> {
    let serial = serialport::new(port, baud)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|e| IoError::Port {
            path: port.to_owned(),
            source: std::io::Error::other(e),
        })?;
    // The factory `usb_to_dynamixel` loop forwards pending bytes before applying a changed USB
    // line-coding baud to Serial1. Give it an empty pass so the first real packet is never sent
    // at the bridge's previous physical baud.
    std::thread::sleep(OPENRB_LINE_CODING_SETTLE);
    let controller = Xl330Controller::new().with_protocol_v2();
    let controller = if fast_sync_read {
        controller.with_fast_sync_read()
    } else {
        controller
    };
    Ok(controller.with_serial_port(serial))
}

/// Which servo a factory-fresh one should become, given the IDs that did not answer.
///
/// Only an unambiguous answer is one: with two servos silent there is no telling which of them
/// the new one replaces, and guessing would flash a leg joint as a neck joint. With none silent
/// there is nothing to adopt — a stray fresh servo on a complete bus is not this code's problem.
pub fn replacement_target(missing: &[u8]) -> Option<u8> {
    match missing {
        [one] => Some(*one),
        _ => None,
    }
}

impl RobotIo for DynamixelIo {
    fn read(&mut self) -> Result<Sensors> {
        self.recover_port_before_read()?;
        let result = self
            .controller
            .sync_read_raw_data(&JOINT_IDS, READ_ADDR, READ_LEN);
        let blocks = result.map_err(|e| self.servo_error("motor sync_read", e))?;

        if blocks.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "sync_read blocks",
                expected: NUM_JOINTS,
                got: blocks.len(),
            });
        }

        let mut sensors = Sensors::default();
        for (joint, block) in blocks.iter().enumerate() {
            if block.len() != READ_LEN as usize {
                return Err(IoError::ShortRead {
                    what: "motor block",
                    expected: READ_LEN as usize,
                    got: block.len(),
                });
            }
            // [0..2] present_pwm, unused · [2..4] current · [4..8] velocity · [8..12] position
            sensors.currents_ma[joint] = (i16::from_le_bytes([block[2], block[3]]) as f64).abs();
            let velocity = i32::from_le_bytes([block[4], block[5], block[6], block[7]]);
            sensors.velocities[joint] = velocity as f64 * RAD_PER_SEC_PER_COUNT;
            let position = i32::from_le_bytes([block[8], block[9], block[10], block[11]]);
            sensors.positions[joint] = (2.0 * PI * position as f64 / 4096.0) - PI;
        }

        let poll = self
            .body_imu
            .poll()
            .map_err(|e| IoError::Bus(format!("read body IMU: {e:#}")))?;
        if let Some(sample) = poll.sample {
            self.last_imu = self.imu.decode(sample);
        }
        let run = self.stale_imu.observe(poll.sample.is_some());
        if run == STALE_RUN_WARN || (run > STALE_RUN_WARN && run.is_multiple_of(500)) {
            tracing::warn!(
                consecutive = run,
                total = self.stale_imu.stale.total,
                "body IMU has produced no new SFLP sample for {run} reads — orientation is frozen"
            );
        }
        if self.stale_imu.frozen() {
            // A couple of empty FIFO polls can be the normal phase difference
            // between the control loop and SFLP. Three in a row is not:
            // accepting the held attitude forever would let a walking policy
            // keep stepping after fusion had stopped. Route a frozen sensor through the same
            // bounded coast/error path as any other required sensor failure;
            // a later fresh FIFO record clears the run and recovers normally.
            return Err(IoError::Bus(format!(
                "body IMU produced no new SFLP sample for {run} consecutive reads"
            )));
        }
        sensors.imu = self.last_imu;

        // This is intentionally the last state change in the method. After USB-link recovery,
        // neither a partial servo answer nor a failed/frozen required IMU may unlock a motion
        // command. It does not restore torque or gains after an OpenRB reset; that is an explicit
        // relax/init operation at the daemon layer.
        self.port_recovery.complete_read();

        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.require_complete_read()?;
        self.controller
            .sync_write_goal_position(&JOINT_IDS, &targets.positions)
            .map_err(|e| self.servo_error("sync_write goal positions", e))
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        // The inherent method, which predates the trait and is still what `robotd init` uses.
        DynamixelIo::set_torque(self, on)
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        self.require_complete_read()?;
        // The status packet is a courtesy the servo may not manage before it resets, so only a
        // failure to send is an error here.
        let result = self.controller.reboot(id);
        result
            .map(|_| ())
            .map_err(|e| self.servo_error(&format!("reboot {id}"), e))
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        self.require_complete_read()?;
        // I and D are written too, at zero — the prototype's `--ki`/`--kd` defaults, which
        // its startup writes to every motor. These are RAM registers, so every power-up
        // restores the servo's factory values, and the factory D gain is not zero: left in
        // place it damps the servo's internal PID, and the robot runs measurably softer
        // than the prototype at the *same* kP. That is not a tuning choice anyone made, so
        // it is pinned here rather than exposed as a knob.
        const KI: u16 = 0;
        const KD: u16 = 0;
        for &id in &JOINT_IDS {
            let result = self.controller.write_position_p_gain(id, kp);
            result.map_err(|e| self.servo_error(&format!("position_p_gain {kp} on {id}"), e))?;
            let result = self.controller.write_position_i_gain(id, KI);
            result.map_err(|e| self.servo_error(&format!("position_i_gain {KI} on {id}"), e))?;
            let result = self.controller.write_position_d_gain(id, KD);
            result.map_err(|e| self.servo_error(&format!("position_d_gain {KD} on {id}"), e))?;
        }
        Ok(())
    }

    /// Supply voltage and case temperatures, in one `sync_read` over registers 144–146.
    ///
    /// Voltage is averaged because all 15 servos sit on one pack: a single reading is the
    /// same measurement with more noise. Temperature is *not* averaged here — the caller gets
    /// every joint, because one loaded joint running hot is the case worth seeing and a mean
    /// over fifteen hides it.
    ///
    /// Note that a silent servo does not produce a short answer: `rustypot`'s `sync_read`
    /// waits for every id and fails the whole transaction if one does not reply. So this is
    /// all-or-nothing, and the caller is expected to keep its previous sample rather than
    /// treat one miss as news. The zero filter on voltage guards a device that answers with a
    /// nonsense value, which must not be averaged in as if the pack were half flat.
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let result = self
            .controller
            .sync_read_raw_data(&JOINT_IDS, SLOW_READ_ADDR, SLOW_READ_LEN);
        let blocks = result.map_err(|e| self.servo_error("voltage+temperature sync_read", e))?;

        if blocks.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "voltage+temperature blocks",
                expected: NUM_JOINTS,
                got: blocks.len(),
            });
        }

        let mut temps_c = [0.0; NUM_JOINTS];
        let mut volts = Vec::with_capacity(NUM_JOINTS);
        for (joint, block) in blocks.iter().enumerate() {
            if block.len() != SLOW_READ_LEN as usize {
                return Err(IoError::ShortRead {
                    what: "voltage+temperature block",
                    expected: SLOW_READ_LEN as usize,
                    got: block.len(),
                });
            }
            // [0..2] present_input_voltage · [2] present_temperature, already in whole °C.
            let counts = u16::from_le_bytes([block[0], block[1]]);
            let v = counts as f64 * VOLTS_PER_COUNT;
            if v > 0.0 {
                volts.push(v);
            }
            temps_c[joint] = block[2] as f64;
        }

        if volts.is_empty() {
            return Err(IoError::ShortRead {
                what: "input voltage",
                expected: NUM_JOINTS,
                got: 0,
            });
        }
        Ok(SlowSensors {
            volts: volts.iter().sum::<f64>() / volts.len() as f64,
            temps_c,
        })
    }

    fn imu_stale(&self) -> ImuStale {
        self.stale_imu.stale
    }

    fn imu_ready(&self) -> bool {
        self.imu.ready()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One silent servo is the only case a swap can be inferred from. With two silent there is
    /// no telling which the fresh servo replaces, and guessing would flash a leg joint as a neck
    /// joint; with none silent a stray fresh servo is nobody's replacement.
    #[test]
    fn a_replacement_is_inferred_only_from_exactly_one_missing_servo() {
        assert_eq!(replacement_target(&[]), None);
        assert_eq!(replacement_target(&[23]), Some(23));
        assert_eq!(replacement_target(&[23, 31]), None);
        assert_eq!(replacement_target(&JOINT_IDS), None);
    }

    /// A servo that simply misses its response deadline is still attached to a valid host port.
    /// Reopening the USB bridge for that case would turn an ordinary dropped packet into a much
    /// longer outage and could hide the actual servo fault.
    #[test]
    fn a_servo_timeout_does_not_arm_usb_recovery() {
        let mut recovery = PortRecovery::default();
        let io_timeout = io::Error::new(io::ErrorKind::TimedOut, "servo did not answer");
        assert!(!recovery.observe(&io_timeout, true));
        assert_eq!(recovery, PortRecovery::Ready);

        let protocol_timeout = rustypot::CommunicationErrorKind::TimeoutError;
        assert!(!recovery.observe(&protocol_timeout, true));
        assert_eq!(recovery, PortRecovery::Ready);
    }

    /// The dependency maps a failed packet send to the same timeout as a silent servo. A missing
    /// stable udev link is the extra fact that makes that otherwise ambiguous error recoverable.
    #[test]
    fn a_send_timeout_arms_recovery_only_when_the_openrb_link_is_gone() {
        let timeout = rustypot::CommunicationErrorKind::TimeoutError;
        assert!(!should_recover_port(&timeout, true));
        assert!(should_recover_port(&timeout, false));
    }

    /// USB removal is different: remember it, reopen only before a read, and keep commands
    /// blocked until that read has produced a complete sample.
    #[test]
    fn an_openrb_disconnect_requires_reopen_then_a_complete_read() {
        let mut recovery = PortRecovery::default();
        let gone = serialport::Error::new(serialport::ErrorKind::NoDevice, "USB device removed");

        assert!(recovery.observe(&gone, false));
        assert!(recovery.needs_reopen());
        assert!(!recovery.may_command());

        recovery.reopened();
        assert!(!recovery.needs_reopen());
        assert!(!recovery.may_command());
        assert!(
            recovery.may_disable_torque(),
            "a live reopened bridge must allow the fail-safe torque-off even before the IMU recovers"
        );

        recovery.complete_read();
        assert!(recovery.may_command());
    }

    /// Linux commonly reports a detached CDC ACM endpoint as `EIO`/`Other` or a broken pipe,
    /// both of which need the same stable-symlink reopen as `serialport::NoDevice`.
    #[test]
    fn host_io_failure_arms_usb_recovery() {
        for kind in [io::ErrorKind::BrokenPipe, io::ErrorKind::Other] {
            let mut recovery = PortRecovery::default();
            let error = io::Error::new(kind, "OpenRB endpoint vanished");
            assert!(
                recovery.observe(&error, true),
                "{kind:?} did not arm recovery"
            );
            assert!(recovery.needs_reopen());
            assert!(
                !recovery.may_disable_torque(),
                "torque-off must not call a controller whose failed reopen left no serial handle"
            );
        }
    }

    /// The block parsed per servo must cover current, velocity and position without
    /// overrunning. If `READ_LEN` and the offsets below ever disagree, joints get each
    /// other's values — which reads as a wiring fault, not a code bug.
    #[test]
    fn read_block_is_long_enough_for_every_field() {
        // Highest offset touched by the parser below is position at 8..12.
        const { assert!(READ_LEN >= 12) };
    }

    /// The conversion must agree with rustypot's own `AnglePosition`, which is what
    /// `sync_write_goal_position` uses on the way out. A mismatch would mean the loop
    /// commands a different angle than it believes it read back.
    #[test]
    fn position_conversion_round_trips_through_rustypot() {
        for raw in [0i32, 1024, 2048, 3072, 4095] {
            let ours = (2.0 * PI * raw as f64 / 4096.0) - PI;
            let theirs = (4096.0 * (PI + ours) / (2.0 * PI)) as i32;
            assert_eq!(theirs, raw, "raw {raw} did not survive the round trip");
        }
    }

    /// 0.229 rev/min per count. Getting this wrong scales every joint velocity in the
    /// observation vector by a constant, which a policy tolerates just well enough to walk
    /// badly.
    #[test]
    fn velocity_scale_matches_the_datasheet_figure() {
        let one_count = RAD_PER_SEC_PER_COUNT;
        let expected_rpm = 0.229;
        assert!((one_count * 60.0 / (2.0 * PI) - expected_rpm).abs() < 1e-12);
    }

    /// A poll without a FIFO quaternion is stale even before the first sample. Readiness keeps
    /// the default orientation out of fall detection, while this counter makes the cause visible.
    #[test]
    fn no_sample_is_stale_from_the_first_poll() {
        let mut t = StaleImuTracker::default();
        assert_eq!(t.observe(false), 1);
        assert_eq!(t.stale.total, 1);
    }

    /// Fresh samples leave the total alone and clear the current run.
    #[test]
    fn fresh_samples_count_for_nothing() {
        let mut t = StaleImuTracker::default();
        for _ in 0..10 {
            assert_eq!(t.observe(true), 0);
        }
        assert_eq!(t.stale, ImuStale { total: 0, run: 0 });
    }

    /// A hiccup: one poll has no new quaternion, then the board recovers. The total remembers it — that
    /// is what makes "9 over 40 minutes" sayable — while the run goes back to zero, because
    /// orientation is live again and nothing should be shouting.
    #[test]
    fn a_hiccup_is_remembered_in_the_total_but_not_the_run() {
        let mut t = StaleImuTracker::default();
        t.observe(true);
        assert_eq!(t.observe(false), 1, "a missed sample starts a run");
        assert_eq!(t.observe(true), 0, "a fresh sample ends the run");
        assert_eq!(t.stale, ImuStale { total: 1, run: 0 });
    }

    /// A board that has stopped refreshing misses forever, and the run is what separates that
    /// from the hiccup above. It has to reach the threshold the journal and the health report
    /// both key off, or a genuinely dead IMU is never reported at all.
    #[test]
    fn a_dead_board_runs_past_the_warning_threshold() {
        let mut t = StaleImuTracker::default();
        for _ in 0..STALE_RUN_WARN - 1 {
            t.observe(false);
        }
        assert!(!t.frozen(), "one early empty poll must still coast");
        t.observe(false);
        assert_eq!(t.stale.run, STALE_RUN_WARN);
        assert_eq!(t.stale.total, STALE_RUN_WARN);
        assert!(t.frozen(), "the threshold must stop stale policy input");
    }

    /// Runs accumulate into the same total across separate episodes: the total is "how often
    /// has this ever happened", not "how bad is it now".
    #[test]
    fn separate_episodes_add_up() {
        let mut t = StaleImuTracker::default();
        for _ in 0..3 {
            t.observe(true);
            t.observe(false);
            t.observe(false);
        }
        assert_eq!(t.stale.total, 6, "two misses in each of three episodes");
        assert_eq!(t.stale.run, 2, "the last episode was still going");
    }
}
