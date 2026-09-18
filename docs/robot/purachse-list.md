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
| Servos | [DYNAMIXEL XL330-M288-T](https://robotis.us/products/dynamixel-xl330-m288-t?variant=51033242960012) | 15 |
| Servo idler components | [ROBOTIS FPX330-H101, 4-piece set](https://robotis.us/products/fpx330-h101-4pcs-set?variant=51033243549836) | 1 set (4 pieces) |
| Side frames | [ROBOTIS FPX330-S102, 4-piece set](https://robotis.us/products/fpx330-s102-4pcs-set?variant=51033242501260) | 1 set (4 pieces) |
| Ball bearings | [McMaster-Carr 6656K181 — 15 mm ID × 22 mm OD × 4 mm thick](https://www.mcmaster.com/6656K181/) | 2 |
| Ball bearings | [McMaster-Carr 6656K68 — 10 mm ID × 15 mm OD × 3 mm thick](https://www.mcmaster.com/6656K68/) | 3 |
| Head 6DoF IMU | [SparkFun 6DoF IMU Breakout — LSM6DSV16X (Qwiic)](https://www.sparkfun.com/sparkfun-6dof-imu-breakout-lsm6dsv16x-qwiic.html) | 1 |
| Body 6DoF IMU | [SparkFun Micro 6DoF IMU Breakout — LSM6DSV16X (Qwiic)](https://www.sparkfun.com/sparkfun-micro-6dof-imu-breakout-lsm6dsv16x-qwiic.html) | 1 |
| 8×8 ToF sensor | [SparkFun Qwiic Mini ToF Imager — VL53L5CX](https://www.sparkfun.com/sparkfun-qwiic-mini-tof-imager-vl53l5cx.html) | 1 |
| Qwiic connection adapter | [SparkFun Qwiic SHIM for Raspberry Pi](https://www.sparkfun.com/sparkfun-qwiic-shim-for-raspberry-pi.html) | 1 |
| Qwiic sensor cables | [SparkFun Qwiic Cable Kit](https://www.sparkfun.com/sparkfun-qwiic-cable-kit.html) | 1 set |
| Camera | [Waveshare IMX219-160 Camera, SKU 16662](https://www.waveshare.com/imx219-160-camera.htm?sku=16662) | 1 |
| NP-F battery | [Amazon battery listing](https://www.amazon.com/dp/B0007Q9PWQ?ref=ppx_yo2ov_dt_b_fed_asin_title) — requested as NP-F500; see note below | 1 |
| NP-F battery adapter plate | [Accsoon Toprig NP-F Battery Adapter Mount Plate](https://www.amazon.com/dp/B0BR6JLLFC?ref=ppx_yo2ov_dt_b_fed_asin_title) | 1 |
| Main computer | [Radxa ZERO 3W](https://radxa.com/products/zeros/zero3w/) | 1 |
