//! Head IMU reader.
//!
//! `tofd` owns this reader because the head IMU and VL53L5CX share the Radxa
//! Qwiic bus. Each has its own `i2c-dev` descriptor; Linux serialises individual
//! transactions on the adapter. The IMU runs on a separate thread so the ToF's
//! firmware upload and retry backoff cannot stall its stream.
//!
//! Gyroscope, accelerometer and quaternion values remain in the sensor's own
//! +X-forward, +Y-left, +Z-up axes. Consumers use the kinematic `head_imu` pose
//! to follow the articulated head and place them in the trunk.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

/// Reopen backoff after an I²C error. A disconnected sensor and a transient bus
/// error look alike here, so one bounded backoff handles both without hammering
/// the shared bus.
const RETRY_MIN: Duration = Duration::from_millis(500);
const RETRY_MAX: Duration = Duration::from_secs(30);
/// A successful chip-ID/configuration exchange is not proof that streaming is
/// healthy. Only reset retry backoff after fresh samples have continued for
/// this long; otherwise an open-success/first-poll-failure cycle hammers the
/// shared bus at the minimum interval forever.
const RETRY_RESET_AFTER: Duration = Duration::from_secs(2);
/// No observed sensor output for this long means the chip is responsive but
/// its sampling path is not streaming. Reopen it instead of advertising a
/// found, frozen IMU.
const NO_ACTIVITY_MIN: Duration = Duration::from_secs(2);

fn no_activity_timeout(hz: u8) -> Duration {
    NO_ACTIVITY_MIN.max(Duration::from_secs_f64(3.0 / f64::from(hz.max(1))))
}

/// How far an IMU subscriber may fall behind before it loses samples. At
/// 100 Hz this is about 2.5 seconds.
pub const FRAME_BUFFER: usize = 256;

/// What `head_imu.stream` reports: which configured IMU was found and the
/// consumer-requested publication rate.
#[derive(Clone)]
pub struct ImuStatus {
    hz: u8,
    inner: Arc<std::sync::Mutex<Inner>>,
}

#[derive(Default)]
struct Inner {
    sensor: Option<String>,
    unavailable: Option<String>,
}

impl ImuStatus {
    pub fn new(hz: u8) -> Self {
        Self {
            hz,
            inner: Arc::new(std::sync::Mutex::new(Inner {
                unavailable: Some("no reading yet".to_owned()),
                ..Inner::default()
            })),
        }
    }

    fn found(&self, sensor: &str) {
        let mut inner = self.inner.lock().unwrap();
        inner.sensor = Some(sensor.to_owned());
        inner.unavailable = None;
    }

    /// Switched off in configuration, rather than absent or broken.
    pub fn off(&self) {
        self.lost(
            "the head IMU is off — `[head_imu] enabled = true` in robotd.toml, then restart tofd"
                .to_owned(),
        );
    }

    /// The selected backend deliberately has no physical head IMU.
    pub fn unavailable(&self, why: &str) {
        self.lost(why.to_owned());
    }

    fn lost(&self, why: String) {
        let mut inner = self.inner.lock().unwrap();
        inner.sensor = None;
        inner.unavailable = Some(why);
    }

    pub fn result(&self) -> proto::HeadImuStreamResult {
        let inner = self.inner.lock().unwrap();
        proto::HeadImuStreamResult {
            accepted: true,
            sensor: inner.sensor.clone(),
            unavailable: inner.unavailable.clone(),
            hz: self.hz,
        }
    }
}

/// Read the explicitly configured head IMU forever and broadcast wire-compatible frames.
pub fn imu_loop(
    bus: &Path,
    address: u8,
    model: qwiic_imu::Model,
    hz: u8,
    status: &ImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    let started = Instant::now();
    let period = Duration::from_secs_f64(1.0 / f64::from(hz.max(1)));
    let mut seq = 0u64;
    let mut backoff = RETRY_MIN;

    while !shutdown.load(Ordering::Acquire) {
        let mut imu = match open_imu(bus, address, model, hz) {
            Ok(found) => found,
            Err(e) => {
                status.lost(e.to_string());
                tracing::warn!(error = %e, backoff_ms = backoff.as_millis(), "no head IMU; will retry");
                sleep_unless_shutdown(backoff, shutdown);
                backoff = (backoff * 2).min(RETRY_MAX);
                continue;
            }
        };
        let sensor_hz = imu.rate_hz();
        let sensor = imu.model().to_string();
        let mut ready = imu.ready();
        tracing::info!(
            bus = %bus.display(),
            address = format!("{address:#04x}"),
            model = %sensor,
            sensor_hz,
            publish_hz = hz,
            ready,
            "head IMU answered"
        );
        let opened_at = Instant::now();
        let mut last_activity_at = opened_at;
        let activity_timeout = no_activity_timeout(hz);
        let mut announced = false;
        let mut retry_reset = false;

        while !shutdown.load(Ordering::Acquire) {
            let tick = Instant::now();
            match imu.poll() {
                Ok(poll) => {
                    if poll.observed {
                        last_activity_at = Instant::now();
                        if !announced {
                            status.found(&sensor);
                            announced = true;
                        }
                        if !retry_reset && opened_at.elapsed() >= RETRY_RESET_AFTER {
                            backoff = RETRY_MIN;
                            retry_reset = true;
                        }
                        let now_ready = imu.ready();
                        if !ready && now_ready {
                            tracing::info!(model = %sensor, "head IMU orientation is ready");
                        } else if ready && !now_ready {
                            tracing::warn!(
                                model = %sensor,
                                "head IMU orientation restarted convergence"
                            );
                        }
                        ready = now_ready;
                    }

                    if let Some(sample) = poll.sample {
                        seq = seq.saturating_add(1);
                        let _ = frames.send(proto::HeadImuFrame {
                            seq,
                            at_us: started.elapsed().as_micros() as u64,
                            t_ns: proto::clock::monotonic_ns(),
                            gyro: sample.gyro,
                            accel: sample.accel,
                            quat: sample.quat,
                            temp_c: sample.temp_c,
                        });
                    } else if !poll.observed && last_activity_at.elapsed() >= activity_timeout {
                        let why = format!(
                            "no head IMU output for {} ms",
                            last_activity_at.elapsed().as_millis()
                        );
                        status.lost(why.clone());
                        tracing::warn!(reason = %why, "head IMU stopped streaming; reopening");
                        break;
                    }
                }
                Err(e) => {
                    status.lost(format!("read failed: {e:#}"));
                    tracing::warn!(error = %e, "head IMU read failed; reopening");
                    break;
                }
            }
            let elapsed = tick.elapsed();
            if elapsed < period {
                sleep_unless_shutdown(period - elapsed, shutdown);
            }
        }

        // A device that answers identification but cannot stream must not be
        // reopened in a tight loop on the bus shared with motor-critical IMU IO.
        sleep_unless_shutdown(backoff, shutdown);
        backoff = (backoff * 2).min(RETRY_MAX);
    }
}

/// Open the fixed-address head IMU on the one configured Qwiic adapter.
fn open_imu(
    bus: &Path,
    address: u8,
    model: qwiic_imu::Model,
    requested_hz: u8,
) -> anyhow::Result<qwiic_imu::Sensor> {
    qwiic_imu::Sensor::open(bus, address, model, u16::from(requested_hz))
        .map_err(|e| anyhow::anyhow!("{model} init at {address:#04x} on {}: {e:#}", bus.display()))
}

fn sleep_unless_shutdown(dur: Duration, shutdown: &Arc<AtomicBool>) {
    // Slice sleeps so shutdown stays prompt during a long retry backoff.
    let slice = Duration::from_millis(50);
    let mut left = dur;
    while left > Duration::ZERO && !shutdown.load(Ordering::Acquire) {
        let step = left.min(slice);
        std::thread::sleep(step);
        left = left.saturating_sub(step);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn switched_off_reads_differently_from_absent() {
        let status = ImuStatus::new(100);

        let fresh = status.result();
        assert!(fresh.sensor.is_none());
        assert_eq!(fresh.hz, 100);

        status.off();
        let off = status.result();
        let why = off.unavailable.expect("a reason");
        assert!(why.contains("[head_imu] enabled"), "{why}");
        assert!(off.sensor.is_none());
        assert!(off.accepted);

        status.lost("nothing answered on any bus".to_owned());
        let absent = status.result();
        let why = absent.unavailable.expect("a reason");
        assert!(!why.contains("[head_imu]"), "{why}");
    }

    #[test]
    fn a_silent_sensor_has_a_finite_rate_aware_deadline() {
        assert_eq!(no_activity_timeout(100), NO_ACTIVITY_MIN);
        assert_eq!(no_activity_timeout(1), Duration::from_secs(3));
        assert_eq!(no_activity_timeout(0), Duration::from_secs(3));
    }
}
