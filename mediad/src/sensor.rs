//! The IMX219 camera profile and media-graph discovery.
//!
//! Replica hardware has one supported head camera: the IMX219 behind a ~3.05 mm M12 lens. The
//! sensor names itself in the topology (`m00_b_imx219 2-0010`); any other sensor is reported by
//! name rather than driven with the IMX219's exposure units and optical model.

use crate::camera::SensorMode;

/// One camera sensor `mediad` knows how to drive.
#[derive(Debug)]
pub struct Sensor {
    /// The readout mode `pipeline` pins, as a media bus format and size.
    pub mode: SensorMode,
    pub bus_format: &'static str,
    pub exposure: Exposure,
    /// Horizontal field of view across a frame in [`Sensor::mode`], degrees.
    pub hfov_deg: Option<f64>,
    /// A solve of this sensor behind its lens, shared by every replica.
    pub family: Option<fn() -> robotd_params::CameraIntrinsics>,
}

impl Sensor {
    /// The substring the driver puts in the media graph's entity name.
    pub const fn name(&self) -> &'static str {
        "imx219"
    }
}

/// What the auto-exposure loop may spend, in this sensor's own units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Exposure {
    /// Shutter spent before any gain, in lines.
    pub soft_lines: f64,
    /// The longest shutter asked for, in lines.
    pub hard_lines: f64,
    /// The analogue gain register value that means 1x.
    pub unity_gain: u32,
    /// Analogue gain ceiling, in multiples of 1x.
    pub max_analogue: f64,
    /// Where the sensor starts, before the loop has metered anything.
    pub start_lines: u32,
    pub start_analogue: f64,
}

impl Exposure {
    /// The starting analogue gain, as the register value.
    pub fn start_gain(&self) -> u32 {
        (self.start_analogue * f64::from(self.unity_gain)) as u32
    }
}

/// The replica's head camera: the IMX219 behind a ~3.05 mm M12 lens.
pub const IMX219: Sensor = Sensor {
    mode: SensorMode {
        width: 1920,
        height: 1080,
    },
    bus_format: "SRGGB10_1X10",
    exposure: Exposure {
        // One line is ~19.05 µs in the pinned mode, which is 1766 lines long.
        soft_lines: 600.0,
        hard_lines: 1200.0,
        unity_gain: 256,
        max_analogue: 11.0,
        start_lines: 600,
        start_analogue: 4.0,
    },
    hfov_deg: Some(62.0),
    family: Some(robotd_params::CameraIntrinsics::alpha),
};

/// Identify the supported sensor in an entity name.
pub fn identify(entity: &str) -> Option<&'static Sensor> {
    entity.contains(IMX219.name()).then_some(&IMX219)
}

/// Choose the IMX219 among sensors found in a media graph, or describe what was found.
pub fn pick(
    found: &[(String, &'static Sensor)],
    others: &[String],
) -> Result<(String, &'static Sensor), String> {
    if let Some((entity, sensor)) = found.first() {
        return Ok((entity.clone(), sensor));
    }
    if others.is_empty() {
        return Err("no sensor in the media graph; expected imx219".to_owned());
    }
    Err(format!(
        "the media graph has {}, but this replica requires imx219",
        others.join(", ")
    ))
}

/// What one media graph holds, read off `media-ctl -p`.
#[derive(Debug, Default)]
pub struct Topology {
    /// Every supported sensor, as entity name and profile.
    pub ours: Vec<(String, &'static Sensor)>,
    /// Sensor entities this daemon does not support.
    pub others: Vec<String>,
}

impl Topology {
    /// Parse `media-ctl -p` output. An entity is a header line followed by its type.
    pub fn read(printed: &str) -> Self {
        let mut topology = Self::default();
        let mut entity: Option<&str> = None;
        for line in printed.lines() {
            let line = line.trim_start();
            if let Some(header) = line.strip_prefix("- entity") {
                entity = header
                    .split_once(": ")
                    .map(|(_, rest)| rest.split(" (").next().unwrap_or(rest).trim())
                    .filter(|name| !name.is_empty());
                if let Some(name) = entity
                    && let Some(sensor) = identify(name)
                {
                    topology.ours.push((name.to_string(), sensor));
                }
                continue;
            }
            if line.starts_with("type ")
                && line.contains("subtype Sensor")
                && let Some(name) = entity.take()
                && identify(name).is_none()
            {
                topology.others.push(name.to_string());
            }
        }
        topology
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_imx219_is_recognised() {
        assert_eq!(identify("m00_b_imx219 2-0010").unwrap().name(), "imx219");
        assert!(identify("m00_b_ov5647 2-0036").is_none());
        assert!(identify("rkisp-isp-subdev").is_none());
    }

    #[test]
    fn another_sensor_is_named_and_refused() {
        let topology = Topology::read(
            "- entity 80: m00_b_ov5647 2-0036 (1 pad, 1 link)\n\
             \x20            type V4L2 subdev subtype Sensor flags 0\n",
        );
        assert!(topology.ours.is_empty());
        assert_eq!(topology.others, ["m00_b_ov5647 2-0036"]);
        let why = pick(&topology.ours, &topology.others).unwrap_err();
        assert!(why.contains("m00_b_ov5647 2-0036"), "{why}");
        assert!(why.contains("requires imx219"), "{why}");
    }

    #[test]
    fn the_imx219_is_accepted() {
        let topology = Topology::read(
            "- entity 76: m00_b_imx219 2-0010 (1 pad, 1 link)\n\
             \x20            type V4L2 subdev subtype Sensor flags 0\n",
        );
        let (entity, sensor) = pick(&topology.ours, &topology.others).unwrap();
        assert_eq!(entity, "m00_b_imx219 2-0010");
        assert_eq!(sensor.name(), "imx219");
    }

    #[test]
    fn the_shutter_caps_have_the_intended_duration() {
        let soft_ms = IMX219.exposure.soft_lines * 19.05 / 1000.0;
        let hard_ms = IMX219.exposure.hard_lines * 19.05 / 1000.0;
        assert!((soft_ms - 11.4).abs() < 0.1);
        assert!((hard_ms - 22.9).abs() < 0.1);
    }

    #[test]
    fn the_starting_gain_uses_imx219_units() {
        assert_eq!(IMX219.exposure.start_gain(), 1024);
    }
}
