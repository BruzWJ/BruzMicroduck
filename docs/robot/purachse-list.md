# Purchase list

Selected off-the-shelf components for one BruzMicroduck replica, replacing the
custom HAT with commercially available boards and sensors.

The design target is to preserve the upstream robot's policy inputs, outputs,
and physical behavior so the existing
[microduck_rl](https://github.com/pollen-robotics/microduck_rl) training pipeline
and exported models remain usable; see the [policy contract](../policy-manifest.md)
and [control-loop design](../design/robotd-design.md).

| Purpose | Component / product link | Quantity |
| --- | --- | --- |
| Servo communication | [ROBOTIS OpenRB-150](https://robotis.us/products/openrb-150) | 1 |
| Host-to-OpenRB data | USB-C data cable | 1 |
| Servo-bus fan-out | [ROBOTIS 3P JST Expansion Board](https://robotis.us/products/3p-jst-expansion-board) | 1 |
| Servos | [DYNAMIXEL XL330-M288-T](https://robotis.us/products/dynamixel-xl330-m288-t?variant=51033242960012) | 15 |
| Servo idler components | [ROBOTIS FPX330-H101, 4-piece set](https://robotis.us/products/fpx330-h101-4pcs-set?variant=51033243549836) | 1 set (4 pieces) |
| Side frames | [ROBOTIS FPX330-S102, 4-piece set](https://robotis.us/products/fpx330-s102-4pcs-set?variant=51033242501260) | 1 set (4 pieces) |
| Head Ball bearings | [McMaster-Carr 6656K181 — 15 mm ID × 22 mm OD × 4 mm thick](https://www.mcmaster.com/6656K181/) | 2 |
| Feet&Mouth Ball bearings | [McMaster-Carr 6656K68 — 10 mm ID × 15 mm OD × 3 mm thick](https://www.mcmaster.com/6656K68/) | 3 |
| Head 6DoF IMU | [SparkFun 6DoF IMU Breakout — LSM6DSV16X (Qwiic)](https://www.sparkfun.com/sparkfun-6dof-imu-breakout-lsm6dsv16x-qwiic.html) | 1 |
| Body 6DoF IMU | [SparkFun Micro 6DoF IMU Breakout — LSM6DSV16X (Qwiic)](https://www.sparkfun.com/sparkfun-micro-6dof-imu-breakout-lsm6dsv16x-qwiic.html) | 1 |
| 8×8 ToF sensor | [SparkFun Qwiic Mini ToF Imager — VL53L5CX](https://www.sparkfun.com/sparkfun-qwiic-mini-tof-imager-vl53l5cx.html) | 1 |
| Qwiic connection adapter | [SparkFun Qwiic SHIM for Raspberry Pi](https://www.sparkfun.com/sparkfun-qwiic-shim-for-raspberry-pi.html) | 1 |
| Qwiic sensor cables | [SparkFun Qwiic Cable Kit](https://www.sparkfun.com/sparkfun-qwiic-cable-kit.html) | 1 set |
| Camera | [Waveshare IMX219-160 Camera, SKU 16662](https://www.waveshare.com/imx219-160-camera.htm?sku=16662) | 1 |
| NP-F battery | [Amazon battery listing](https://www.amazon.com/dp/B0007Q9PWQ?ref=ppx_yo2ov_dt_b_fed_asin_title) — requested as NP-F500; see note below | 1 |
| NP-F battery adapter plate | [Accsoon Toprig NP-F Battery Adapter Mount Plate](https://www.amazon.com/dp/B0BR6JLLFC?ref=ppx_yo2ov_dt_b_fed_asin_title) | 1 |
| Main computer | [Radxa ZERO 3W](https://radxa.com/products/zeros/zero3w/) | 1 |

The complete OpenRB firmware, power, wiring and servo-ID contract is in
[`robotd-design.md` §1.1](../design/robotd-design.md#11-the-two-buses-and-who-owns-them).

## Qwiic assembly

Fit the Qwiic SHIM to the Radxa's Pi-style header with its pin-1 mark aligned. The SHIM uses
header pins 3/5 for SDA/SCL and regulates the header's 5 V supply to the Qwiic chain's 3.3 V;
installing it backwards can short the supply.

Connect the boards in this order, starting at the Radxa:

```text
Qwiic SHIM -> VL53L5CX ToF -> head standard LSM6DSV16X -> body Micro LSM6DSV16X
```

That order is structural, not cosmetic. The ToF and standard IMU each have two Qwiic connectors
and pass the bus onward. The Micro IMU has only one connector, so it must be the endpoint; it
cannot be the first board in a connector-only daisy chain.

Mount both SparkFun IMUs with their labelled axes in the robot convention: **+X forward, +Y left,
+Z up**. The body board's sensor axes then equal the trunk axes directly. The head board uses the
same convention in the neutral head pose; its MJCF site follows those axes through the articulated
head. A bracket that rotates either breakout also requires the matching transform change in
`duck-control/src/imu.rs` or the head MJCF site; wiring alone cannot correct a rotated sensor.

The replacement bracket has not been measured in this repository, so the current alpha MJCF poses
remain the mechanical contract rather than a claim that the SparkFun hole pattern lands there
automatically: `tof` is at `pos="0.0143 0.0225 -0.0735"` with
`quat="0.707107 0 0.707107 0"`, and `head_imu` is at
`pos="0.0114823 0.000202447 -0.05126"` with the same quaternion, both in the
`bottom_head_shell` frame. The shell body's own asset transform remains unchanged; the site-local
quaternion expresses the physically forward/up sensor mounting in that parent frame. Their
composition makes the neutral sensor frame +X forward, +Y left, +Z up. The depth convention is
likewise +X optical-forward, +Y sensor-left, +Z up; wire zone 0 is the top-left return. If the
bracket differs, update `kinematics/assets/alpha/robot_walk.xml` before mapping and verify all four
grid corners against a flat target on hardware.

Prepare the addresses before assembly:

- Leave the body Micro IMU at its factory Linux 7-bit address, `0x6b`.
- On the head IMU, cut the ADDR jumper's power-side trace and bridge its centre pad to ground,
  selecting `0x6a`. Do not leave the address pad floating or open both sides into SPI mode.
- Leave the VL53L5CX at `0x29`. ST material also writes `0x52`/`0x53`, but those are the shifted
  8-bit write/read forms; Linux `i2c-dev` and this repository use `0x29`.

Every SparkFun sensor board supplies a 2.2 kΩ SDA/SCL pull-up pair. Three enabled pairs in
parallel are already about 733 Ω, before the Radxa's header-side pull-ups, which is too strong.
For this build, cut the I2C pull-up jumper on all three sensor boards and use the Radxa-side
pull-ups. If the host wiring changes, retain at most one sensor-board pair and verify the effective
resistance and rise time rather than enabling all of them.

Provisioning, the `/dev/i2c-qwiic` name, and the I2C3/FUSB302 pinmux consequence are owned by
[`deploy/README.md`](../../deploy/README.md#what-those-commands-actually-do).
