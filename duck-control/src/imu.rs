//! Body-frame interpretation of the trunk LSM6DSV16X.
//!
//! [`qwiic_imu`] owns the Linux I²C device and the chip's SFLP configuration. This module
//! keeps the robot-specific part: the sensor-to-trunk mounting transform, spike rejection,
//! projected gravity, and the readiness gate used by fall detection.

use crate::model::NUM_JOINTS;
use qwiic_imu::Sample;

/// What the control loop knows about the robot's orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    /// Angular velocity in the trunk frame, rad/s.
    pub gyro: [f64; 3],
    /// Projected gravity in the trunk frame, unit vector. Upright is `[0, 0, -1]`.
    ///
    /// This is what the policy observes, and what fall detection thresholds on.
    pub gravity: [f64; 3],
    /// Orientation, trunk→world, scalar-first `[w, x, y, z]`.
    pub quat: [f64; 4],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            gravity: [0.0, 0.0, -1.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// Transforms fused sensor samples into [`ImuData`].
///
/// Stateful only for spike rejection and the readiness count; SFLP runs in the chip. Keeping
/// the robot-frame transform here means both physical IMU roles can share the hardware driver
/// without pretending they have the same mounting orientation.
pub struct SflpDecoder {
    /// Sensor→trunk mounting rotation, scalar-first.
    mount: [f64; 4],
    /// Live SFLP samples. Gates [`SflpDecoder::ready`].
    quat_samples: u32,
    gyro_history: [[f64; 3]; 2],
    gravity_history: [[f64; 3]; 2],
}

impl Default for SflpDecoder {
    fn default() -> Self {
        Self::new(Self::DEFAULT_MOUNT)
    }
}

impl SflpDecoder {
    /// The board is mounted so that trunk = `[+raw_z, +raw_y, −raw_x]`, a +90° rotation
    /// about Y.
    pub const DEFAULT_MOUNT: [f64; 4] = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];

    pub fn new(mount: [f64; 4]) -> Self {
        Self {
            mount,
            quat_samples: 0,
            gyro_history: [[0.0; 3]; 2],
            gravity_history: [[0.0, 0.0, -1.0]; 2],
        }
    }

    /// Whether the chip has demonstrably produced fused output — roughly 0.5 s at 50 Hz.
    ///
    /// Until this is true the orientation is a default, not a measurement. Slice 2's fall
    /// detection must not run before it.
    pub fn ready(&self) -> bool {
        self.quat_samples >= 25
    }

    pub fn decode(&mut self, sample: Sample) -> ImuData {
        let gyro_sensor = sample.gyro.map(f64::from);
        let gyro = rotate(self.mount, gyro_sensor);

        let sensor_quat = sample.quat.map(f64::from);
        let mount_inv = [
            self.mount[0],
            -self.mount[1],
            -self.mount[2],
            -self.mount[3],
        ];
        let q = mul(sensor_quat, mount_inv);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        // The shared driver rejects invalid FIFO quaternions. Normalising here only removes
        // conversion and multiplication rounding before the value reaches the policy.
        let quat = [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm];
        self.quat_samples = self.quat_samples.saturating_add(1);

        // Normalise *before* the median, matching the runtime. Note the consequence: a
        // component-wise median across three unit vectors is not itself unit-norm, so
        // during a transient the policy sees a slightly short vector. Steady state is
        // exact. Whether the training env expects strict unit norm is worth settling when
        // slice 2 wires up observations — normalising after the median would guarantee it,
        // but that is a behaviour change to a path that currently walks.
        let gravity = normalise(rotate_inverse(quat, [0.0, 0.0, -1.0]));

        let out = ImuData {
            gyro: median3_each(&self.gyro_history, gyro),
            gravity: median3_each(&self.gravity_history, gravity),
            quat,
        };
        self.gyro_history = [self.gyro_history[1], gyro];
        self.gravity_history = [self.gravity_history[1], gravity];
        out
    }
}

/// Single-sample spike rejection. A dropped or corrupted block shows up as one wild value;
/// a median over three discards it without the lag of an average.
fn median3_each(history: &[[f64; 3]; 2], now: [f64; 3]) -> [f64; 3] {
    let m = |a: f64, b: f64, c: f64| a.max(b).min(c).max(a.min(b));
    [
        m(history[0][0], history[1][0], now[0]),
        m(history[0][1], history[1][1], now[1]),
        m(history[0][2], history[1][2], now[2]),
    ]
}

/// Hamilton product, scalar-first.
fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// `q · v · q⁻¹`
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] + q[0] * t[0] + c[0],
        v[1] + q[0] * t[1] + c[1],
        v[2] + q[0] * t[2] + c[2],
    ]
}

/// `q⁻¹ · v · q` — world vector expressed in the body frame.
fn rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] - q[0] * t[0] + c[0],
        v[1] - q[0] * t[1] + c[1],
        v[2] - q[0] * t[2] + c[2],
    ]
}

fn cross_terms(q: [f64; 4], v: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (x, y, z) = (q[1], q[2], q[3]);
    let t = [
        (y * v[2] - z * v[1]) * 2.0,
        (z * v[0] - x * v[2]) * 2.0,
        (x * v[1] - y * v[0]) * 2.0,
    ];
    let c = [
        y * t[2] - z * t[1],
        z * t[0] - x * t[2],
        x * t[1] - y * t[0],
    ];
    (t, c)
}

/// Unit vector, falling back to "upright" rather than dividing by ~zero.
fn normalise(v: [f64; 3]) -> [f64; 3] {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag > 0.1 {
        [v[0] / mag, v[1] / mag, v[2] / mag]
    } else {
        [0.0, 0.0, -1.0]
    }
}

/// Compile-time assurance that the joint count and this module stay independent.
const _: () = assert!(NUM_JOINTS > 0);

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sequence: u64, gyro: [f32; 3], quat: [f32; 4]) -> Sample {
        Sample {
            sequence,
            gyro,
            accel: [0.0; 3],
            quat,
            temp_c: 25.0,
        }
    }

    /// `ready()` gates fall detection in slice 2. If it were true from the first block,
    /// the robot would be judged on a default orientation for the first quarter second.
    #[test]
    fn not_ready_until_the_chip_has_produced_output() {
        let mut d = SflpDecoder::default();
        let mounted = SflpDecoder::DEFAULT_MOUNT.map(|v| v as f32);
        for sequence in 1..=24 {
            d.decode(sample(sequence, [0.0; 3], mounted));
        }
        assert!(!d.ready());
        d.decode(sample(25, [0.0; 3], mounted));
        assert!(d.ready());
    }

    /// The physical mounting convention is part of the trained robot, not a property of the
    /// SparkFun board. Changing breakouts must not rotate the policy's gyro axes.
    #[test]
    fn body_mount_maps_sensor_axes_into_the_trunk() {
        let mut d = SflpDecoder::default();
        let mounted = SflpDecoder::DEFAULT_MOUNT.map(|v| v as f32);
        let s = sample(1, [1.0, 2.0, 3.0], mounted);
        // Three identical samples let the component median settle.
        d.decode(s);
        d.decode(s);
        let out = d.decode(s);
        assert!((out.gyro[0] - 3.0).abs() < 1e-6);
        assert!((out.gyro[1] - 2.0).abs() < 1e-6);
        assert!((out.gyro[2] + 1.0).abs() < 1e-6);
        assert!(out.gravity[0].abs() < 1e-6);
        assert!(out.gravity[1].abs() < 1e-6);
        assert!((out.gravity[2] + 1.0).abs() < 1e-6);
        assert!((out.quat[0] - 1.0).abs() < 1e-6);
    }

    /// In steady state gravity must be a unit vector at any orientation — the policy
    /// observes it directly and was trained on normalised input.
    ///
    /// Three identical blocks per orientation so the median settles. Mid-transient the
    /// median blends three different unit vectors component-wise and the result is
    /// slightly short; that is the runtime's behaviour and is deliberately preserved (see
    /// the note in `decode`).
    #[test]
    fn gravity_is_a_unit_vector_in_steady_state() {
        let mut d = SflpDecoder::default();
        for quat in [
            [1.0, 0.0, 0.0, 0.0],
            [0.923_879_5, 0.382_683_4, 0.0, 0.0],
            [0.923_879_5, 0.0, 0.382_683_4, 0.0],
            [0.923_879_5, 0.0, 0.0, -0.382_683_4],
        ] {
            let s = sample(1, [0.0; 3], quat);
            d.decode(s);
            d.decode(s);
            let g = d.decode(s).gravity;
            let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
            assert!((mag - 1.0).abs() < 1e-9, "gravity magnitude {mag}");
        }
    }

    /// The transient case, pinned so the bound is known rather than assumed: switching
    /// orientation between ticks blends three vectors and shortens the result. If this
    /// ever gets far from 1.0 the policy is being fed something training never saw.
    #[test]
    fn gravity_stays_close_to_unit_through_a_transient() {
        let mut d = SflpDecoder::default();
        d.decode(sample(1, [0.0; 3], [1.0, 0.0, 0.0, 0.0]));
        let g = d
            .decode(sample(2, [0.0; 3], [0.923_879_5, 0.382_683_4, 0.0, 0.0]))
            .gravity;
        let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
        assert!(mag > 0.5, "transient gravity collapsed to {mag}");
        assert!(mag <= 1.0 + 1e-9);
    }

    /// The shared driver supplies signed SI units. Keep the sign through the robot mount.
    #[test]
    fn gyro_sign_is_preserved() {
        let mut d = SflpDecoder::default();
        let mounted = SflpDecoder::DEFAULT_MOUNT.map(|v| v as f32);
        let s = sample(1, [-1.25, 0.0, 0.0], mounted);
        d.decode(s);
        d.decode(s);
        let out = d.decode(s);
        assert!((out.gyro[2] - 1.25).abs() < 1e-9);
    }
}
