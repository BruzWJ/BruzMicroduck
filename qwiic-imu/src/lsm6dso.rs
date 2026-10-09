//! Raw FIFO backend for an explicitly selected LSM6DSO.
//!
//! The LSM6DSO is not an LSM6DSV16X variant: it identifies as `0x6c`, has
//! different output-rate registers, and has no SFLP quaternion engine. This
//! backend therefore drains every ordered accelerometer/gyroscope pair for the
//! common [`crate::Sensor`] to fuse in software.

use crate::{ACCEL_MPS2_PER_COUNT, GYRO_RAD_PER_COUNT};

/// The identity returned by the LSM6DSO `WHO_AM_I` register.
pub(crate) const ID: u8 = 0x6c;

/// One raw FIFO time slot, converted to SI units in the sensor's own axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RawSample {
    /// Complete accel/gyro pairs produced since this sensor was opened.
    pub sequence: u64,
    /// Angular velocity, rad/s.
    pub gyro: [f32; 3],
    /// Specific force, m/s².
    pub accel: [f32; 3],
}

/// Everything drained by one bounded FIFO poll.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Batch {
    /// Ordered samples. Empty means that no complete new pair was available.
    pub samples: Vec<RawSample>,
    /// Most recently read die temperature, °C.
    pub temp_c: f32,
}

#[derive(Debug, Clone, Copy)]
struct Rate {
    hz: f32,
    register: u8,
}

fn rate(requested_hz: u16) -> Rate {
    match requested_hz {
        0..=12 => Rate {
            hz: 12.5,
            register: 0x01,
        },
        13..=26 => Rate {
            hz: 26.0,
            register: 0x02,
        },
        27..=52 => Rate {
            hz: 52.0,
            register: 0x03,
        },
        53..=104 => Rate {
            hz: 104.0,
            register: 0x04,
        },
        105..=208 => Rate {
            hz: 208.0,
            register: 0x05,
        },
        _ => Rate {
            hz: 416.0,
            register: 0x06,
        },
    }
}

#[cfg(target_os = "linux")]
mod linux {
    use std::fmt::Debug;
    use std::time::Duration;

    use anyhow::{Context, Result, anyhow, bail};
    use embedded_hal::i2c::I2c;
    use linux_embedded_hal::I2cdev;

    use super::{ACCEL_MPS2_PER_COUNT, Batch, GYRO_RAD_PER_COUNT, ID, Rate, RawSample, rate};

    const WHO_AM_I: u8 = 0x0f;
    const CTRL1_XL: u8 = 0x10;
    const CTRL2_G: u8 = 0x11;
    const CTRL3_C: u8 = 0x12;
    const CTRL9_XL: u8 = 0x18;
    const OUT_TEMP_L: u8 = 0x20;
    const FIFO_CTRL1: u8 = 0x07;
    const FIFO_CTRL2: u8 = 0x08;
    const FIFO_CTRL3: u8 = 0x09;
    const FIFO_CTRL4: u8 = 0x0a;
    const FIFO_STATUS1: u8 = 0x3a;
    const FIFO_DATA_OUT_TAG: u8 = 0x78;

    const SW_RESET: u8 = 1 << 0;
    const IF_INC: u8 = 1 << 2;
    const BDU: u8 = 1 << 6;
    const I3C_DISABLE: u8 = 1 << 1;

    const FIFO_BYPASS: u8 = 0x00;
    const FIFO_STREAM: u8 = 0x06;
    const FIFO_OVERRUN: u8 = 1 << 6;
    const FIFO_OVERRUN_LATCHED: u8 = 1 << 3;
    const FIFO_WATERMARK_RECORDS: u8 = 2;
    /// At the minimum 12.5 Hz ODR, the slowest supported 1 Hz caller can see 13
    /// paired time slots, or 26 records. Thirty-two leaves scheduling margin
    /// while bounding the drain to under roughly 10 ms on a 400 kHz Qwiic bus,
    /// still inside the normal 20 ms control period.
    const MAX_FIFO_RECORDS: u16 = 32;

    const TAG_GYRO: u8 = 0x01;
    const TAG_ACCEL: u8 = 0x02;

    const BOOT_DELAY: Duration = Duration::from_millis(10);
    const RESET_ATTEMPTS: usize = 100;
    const RESET_POLL_DELAY: Duration = Duration::from_millis(1);

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum SensorTag {
        Gyro,
        Accel,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct FifoTag {
        sensor: SensorTag,
        slot: u8,
    }

    impl FifoTag {
        fn decode(raw: u8) -> Result<Self> {
            // ST's FIFO utility defines TAG_PARITY as even parity across the
            // complete tag byte, including the parity bit itself.
            if !raw.count_ones().is_multiple_of(2) {
                bail!("FIFO tag {raw:#04x} failed its even-parity check");
            }

            let sensor = match raw >> 3 {
                TAG_GYRO => SensorTag::Gyro,
                TAG_ACCEL => SensorTag::Accel,
                tag => bail!("unexpected FIFO sensor tag {tag:#04x}"),
            };
            Ok(Self {
                sensor,
                slot: (raw >> 1) & 0x03,
            })
        }
    }

    struct PendingPair {
        slot: Option<u8>,
        last_complete_slot: Option<u8>,
        gyro: Option<[f32; 3]>,
        accel: Option<[f32; 3]>,
    }

    impl PendingPair {
        fn new() -> Self {
            Self {
                slot: None,
                last_complete_slot: None,
                gyro: None,
                accel: None,
            }
        }

        fn clear(&mut self) {
            self.slot = None;
            self.last_complete_slot = None;
            self.gyro = None;
            self.accel = None;
        }

        fn push(&mut self, tag: FifoTag, raw: [i16; 3]) -> Result<Option<([f32; 3], [f32; 3])>> {
            match self.slot {
                Some(slot) if slot != tag.slot => bail!(
                    "FIFO time slot changed from {slot} to {} before its accel/gyro pair was complete",
                    tag.slot
                ),
                Some(_) => {}
                None => {
                    if let Some(previous) = self.last_complete_slot {
                        let expected = (previous + 1) & 0x03;
                        if tag.slot != expected {
                            bail!(
                                "FIFO time slot jumped from {previous} to {}; expected {expected}",
                                tag.slot
                            );
                        }
                    }
                    self.slot = Some(tag.slot);
                }
            }

            match tag.sensor {
                SensorTag::Gyro => {
                    if self.gyro.replace(raw.map(gyro_to_si)).is_some() {
                        bail!("duplicate gyroscope record in FIFO time slot {}", tag.slot);
                    }
                }
                SensorTag::Accel => {
                    if self.accel.replace(raw.map(accel_to_si)).is_some() {
                        bail!(
                            "duplicate accelerometer record in FIFO time slot {}",
                            tag.slot
                        );
                    }
                }
            }

            match (self.gyro.take(), self.accel.take()) {
                (Some(gyro), Some(accel)) => {
                    self.slot = None;
                    self.last_complete_slot = Some(tag.slot);
                    Ok(Some((gyro, accel)))
                }
                (gyro, accel) => {
                    self.gyro = gyro;
                    self.accel = accel;
                    Ok(None)
                }
            }
        }
    }

    fn gyro_to_si(raw: i16) -> f32 {
        f32::from(raw) * GYRO_RAD_PER_COUNT
    }

    fn accel_to_si(raw: i16) -> f32 {
        f32::from(raw) * ACCEL_MPS2_PER_COUNT
    }

    struct Driver<I2C> {
        i2c: I2C,
        address: u8,
        rate: Rate,
        sequence: u64,
        next_temp_sequence: u64,
        temp_every: u64,
        temp_c: f32,
        pending: PendingPair,
    }

    impl<I2C> Driver<I2C>
    where
        I2C: I2c,
        I2C::Error: Debug,
    {
        fn initialise(
            i2c: I2C,
            address: u8,
            requested_hz: u16,
            mut delay: impl FnMut(Duration),
        ) -> Result<Self> {
            if !matches!(address, 0x6a | 0x6b) {
                bail!("LSM6DSO address must be 0x6a or 0x6b, got {address:#04x}");
            }

            let rate = rate(requested_hz);
            let mut driver = Self {
                i2c,
                address,
                rate,
                sequence: 0,
                next_temp_sequence: 1,
                temp_every: rate.hz.ceil() as u64,
                temp_c: 25.0,
                pending: PendingPair::new(),
            };

            delay(BOOT_DELAY);
            let id = driver.read_register(WHO_AM_I)?;
            if id != ID {
                bail!(
                    "device at {address:#04x} has WHO_AM_I {id:#04x}, expected {ID:#04x} for LSM6DSO"
                );
            }

            driver.update_register(CTRL3_C, SW_RESET, SW_RESET)?;
            let mut reset_complete = false;
            for _ in 0..RESET_ATTEMPTS {
                if driver.read_register(CTRL3_C)? & SW_RESET == 0 {
                    reset_complete = true;
                    break;
                }
                delay(RESET_POLL_DELAY);
            }
            if !reset_complete {
                bail!("LSM6DSO reset did not finish within 100ms");
            }

            // ST recommends disabling I3C during initialisation when this is an
            // ordinary I²C bus. BDU and IF_INC make each multibyte FIFO/output
            // transaction coherent and explicit rather than relying on reset.
            driver.update_register(CTRL9_XL, I3C_DISABLE, I3C_DISABLE)?;
            driver.update_register(CTRL3_C, BDU | IF_INC, BDU | IF_INC)?;

            // Configure FIFO while both sensing chains are still powered down.
            // Each time slot is two records: one gyro and one accelerometer.
            driver.write_register(FIFO_CTRL4, FIFO_BYPASS)?;
            driver.write_register(FIFO_CTRL1, FIFO_WATERMARK_RECORDS)?;
            driver.write_register(FIFO_CTRL2, 0)?;
            driver.write_register(FIFO_CTRL3, (rate.register << 4) | rate.register)?;
            driver.write_register(FIFO_CTRL4, FIFO_STREAM)?;

            // One write per sensor selects ODR and full scale atomically:
            // accelerometer FS=10 is ±4 g; gyro FS=01 is ±500 dps.
            driver.write_register(CTRL1_XL, (rate.register << 4) | 0x08)?;
            driver.write_register(CTRL2_G, (rate.register << 4) | 0x04)?;

            Ok(driver)
        }

        fn read_register(&mut self, register: u8) -> Result<u8> {
            Ok(self.read_registers::<1>(register)?[0])
        }

        fn read_registers<const N: usize>(&mut self, register: u8) -> Result<[u8; N]> {
            let mut value = [0; N];
            self.i2c
                .write_read(self.address, &[register], &mut value)
                .map_err(|e| anyhow!("read LSM6DSO register {register:#04x}: {e:?}"))?;
            Ok(value)
        }

        fn write_register(&mut self, register: u8, value: u8) -> Result<()> {
            self.i2c
                .write(self.address, &[register, value])
                .map_err(|e| anyhow!("write LSM6DSO register {register:#04x}: {e:?}"))
        }

        fn update_register(&mut self, register: u8, mask: u8, value: u8) -> Result<()> {
            let current = self.read_register(register)?;
            let updated = (current & !mask) | (value & mask);
            if updated != current {
                self.write_register(register, updated)?;
            }
            Ok(())
        }

        fn restart_fifo(&mut self) -> Result<()> {
            self.write_register(FIFO_CTRL4, FIFO_BYPASS)?;
            self.write_register(FIFO_CTRL4, FIFO_STREAM)?;
            self.pending.clear();
            Ok(())
        }

        fn poll(&mut self) -> Result<Batch> {
            let status = self.read_registers::<2>(FIFO_STATUS1)?;
            let level = u16::from(status[0]) | (u16::from(status[1] & 0x03) << 8);
            let overrun = status[1] & (FIFO_OVERRUN | FIFO_OVERRUN_LATCHED) != 0;
            if overrun || level > MAX_FIFO_RECORDS {
                self.restart_fifo()?;
                bail!("LSM6DSO FIFO backlog is {level} records (overrun={overrun}); discarded it");
            }

            let mut samples = Vec::with_capacity(usize::from(level.div_ceil(2)));
            for _ in 0..level {
                let record = self.read_registers::<7>(FIFO_DATA_OUT_TAG)?;
                let tag = match FifoTag::decode(record[0]) {
                    Ok(tag) => tag,
                    Err(error) => {
                        self.restart_fifo()?;
                        return Err(error.context("corrupt LSM6DSO FIFO metadata; discarded FIFO"));
                    }
                };
                let raw = [
                    i16::from_le_bytes([record[1], record[2]]),
                    i16::from_le_bytes([record[3], record[4]]),
                    i16::from_le_bytes([record[5], record[6]]),
                ];
                match self.pending.push(tag, raw) {
                    Ok(Some((gyro, accel))) => {
                        self.sequence = self.sequence.saturating_add(1);
                        samples.push(RawSample {
                            sequence: self.sequence,
                            gyro,
                            accel,
                        });
                    }
                    Ok(None) => {}
                    Err(error) => {
                        self.restart_fifo()?;
                        return Err(error.context("corrupt LSM6DSO FIFO sequence; discarded FIFO"));
                    }
                }
            }

            if self.sequence >= self.next_temp_sequence && !samples.is_empty() {
                let raw = i16::from_le_bytes(self.read_registers::<2>(OUT_TEMP_L)?);
                self.temp_c = f32::from(raw) / 256.0 + 25.0;
                self.next_temp_sequence = self.sequence.saturating_add(self.temp_every);
            }

            Ok(Batch {
                samples,
                temp_c: self.temp_c,
            })
        }
    }

    /// One explicitly selected LSM6DSO with its own `i2c-dev` descriptor.
    pub(crate) struct Sensor {
        driver: Driver<I2cdev>,
    }

    impl Sensor {
        pub(crate) fn open(bus: &std::path::Path, address: u8, requested_hz: u16) -> Result<Self> {
            let i2c = I2cdev::new(bus).with_context(|| format!("open {}", bus.display()))?;
            let driver = Driver::initialise(i2c, address, requested_hz, std::thread::sleep)?;
            Ok(Self { driver })
        }

        /// Actual accel/gyro FIFO rate after rounding to an LSM6DSO ODR rung.
        pub(crate) fn rate_hz(&self) -> f32 {
            self.driver.rate.hz
        }

        /// Drain all complete FIFO pairs, in sensor order.
        pub(crate) fn poll(&mut self) -> Result<Batch> {
            self.driver.poll()
        }
    }

    #[cfg(test)]
    mod tests {
        use std::collections::VecDeque;
        use std::convert::Infallible;

        use embedded_hal::i2c::{ErrorType, I2c, Operation, SevenBitAddress};

        use super::*;

        struct FakeI2c {
            registers: [u8; 256],
            fifo: VecDeque<[u8; 7]>,
            reset_reads: u8,
        }

        impl FakeI2c {
            fn lsm6dso() -> Self {
                let mut registers = [0; 256];
                registers[usize::from(WHO_AM_I)] = ID;
                registers[usize::from(CTRL3_C)] = IF_INC;
                Self {
                    registers,
                    fifo: VecDeque::new(),
                    reset_reads: 0,
                }
            }

            fn record(tag: u8, slot: u8, xyz: [i16; 3]) -> [u8; 7] {
                let mut record = [0; 7];
                let tag_without_parity = (tag << 3) | ((slot & 0x03) << 1);
                record[0] = tag_without_parity | (tag_without_parity.count_ones() as u8 & 1);
                for (index, value) in xyz.into_iter().enumerate() {
                    let start = 1 + index * 2;
                    record[start..start + 2].copy_from_slice(&value.to_le_bytes());
                }
                record
            }

            fn write_impl(&mut self, write: &[u8]) {
                assert!(write.len() >= 2, "register plus at least one value");
                let start = usize::from(write[0]);
                for (offset, value) in write[1..].iter().copied().enumerate() {
                    self.registers[start + offset] = value;
                }
                if write[0] == CTRL3_C && write[1] & SW_RESET != 0 {
                    self.reset_reads = 2;
                }
                if write[0] == FIFO_CTRL4 && write[1] & 0x07 == FIFO_BYPASS {
                    self.fifo.clear();
                }
            }

            fn read_impl(&mut self, register: u8, read: &mut [u8]) {
                if register == FIFO_STATUS1 && read.len() == 2 {
                    let level = self.fifo.len() as u16;
                    read[0] = level as u8;
                    read[1] = (self.registers[usize::from(FIFO_STATUS1 + 1)] & !0x03)
                        | ((level >> 8) as u8 & 0x03);
                    return;
                }
                if register == FIFO_DATA_OUT_TAG && read.len() == 7 {
                    read.copy_from_slice(&self.fifo.pop_front().expect("FIFO record"));
                    return;
                }
                if register == CTRL3_C && self.reset_reads != 0 {
                    self.reset_reads -= 1;
                    if self.reset_reads == 0 {
                        self.registers[usize::from(CTRL3_C)] &= !SW_RESET;
                    }
                }
                let start = usize::from(register);
                read.copy_from_slice(&self.registers[start..start + read.len()]);
            }
        }

        impl ErrorType for FakeI2c {
            type Error = Infallible;
        }

        impl I2c<SevenBitAddress> for FakeI2c {
            fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
                assert_eq!(address, 0x6a);
                self.write_impl(write);
                Ok(())
            }

            fn write_read(
                &mut self,
                address: u8,
                write: &[u8],
                read: &mut [u8],
            ) -> Result<(), Self::Error> {
                assert_eq!(address, 0x6a);
                assert_eq!(write.len(), 1);
                self.read_impl(write[0], read);
                Ok(())
            }

            fn transaction(
                &mut self,
                _address: u8,
                _operations: &mut [Operation<'_>],
            ) -> Result<(), Self::Error> {
                unreachable!("the driver uses write and write_read")
            }
        }

        fn driver(requested_hz: u16) -> Driver<FakeI2c> {
            Driver::initialise(FakeI2c::lsm6dso(), 0x6a, requested_hz, |_| {})
                .expect("initialise fake LSM6DSO")
        }

        #[test]
        fn initialises_the_documented_fifo_ranges_and_rate() {
            let driver = driver(100);
            assert_eq!(driver.rate.hz, 104.0);
            assert_eq!(driver.i2c.registers[usize::from(CTRL3_C)], BDU | IF_INC);
            assert_eq!(driver.i2c.registers[usize::from(CTRL9_XL)], I3C_DISABLE);
            assert_eq!(driver.i2c.registers[usize::from(FIFO_CTRL3)], 0x44);
            assert_eq!(driver.i2c.registers[usize::from(FIFO_CTRL4)], FIFO_STREAM);
            assert_eq!(driver.i2c.registers[usize::from(CTRL1_XL)], 0x48);
            assert_eq!(driver.i2c.registers[usize::from(CTRL2_G)], 0x44);
        }

        #[test]
        fn drains_every_complete_pair_in_order_and_converts_units() {
            let mut driver = driver(100);
            driver.i2c.registers[usize::from(OUT_TEMP_L)] = 0x00;
            driver.i2c.registers[usize::from(OUT_TEMP_L + 1)] = 0x01;
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_GYRO, 0, [100, -200, 300]));
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_ACCEL, 0, [400, -500, 600]));
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_ACCEL, 1, [700, 800, -900]));
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_GYRO, 1, [-1000, 1100, 1200]));

            let batch = driver.poll().expect("drain FIFO");
            assert_eq!(batch.samples.len(), 2);
            assert_eq!(batch.samples[0].sequence, 1);
            assert_eq!(batch.samples[1].sequence, 2);
            assert_eq!(batch.samples[0].gyro[0], 100.0 * GYRO_RAD_PER_COUNT);
            assert_eq!(batch.samples[0].accel[1], -500.0 * ACCEL_MPS2_PER_COUNT);
            assert_eq!(batch.samples[1].gyro[0], -1000.0 * GYRO_RAD_PER_COUNT);
            assert_eq!(batch.samples[1].accel[2], -900.0 * ACCEL_MPS2_PER_COUNT);
            assert_eq!(batch.temp_c, 26.0);
        }

        #[test]
        fn retains_an_incomplete_pair_across_polls() {
            let mut driver = driver(100);
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_GYRO, 3, [1, 2, 3]));
            let first = driver.poll().expect("first half");
            assert!(first.samples.is_empty());

            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_ACCEL, 3, [4, 5, 6]));
            let batch = driver.poll().expect("second half");
            assert_eq!(batch.samples.len(), 1);
            assert_eq!(batch.samples[0].sequence, 1);
        }

        #[test]
        fn refuses_to_pair_records_from_different_time_slots() {
            let mut driver = driver(100);
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_GYRO, 0, [1, 2, 3]));
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_ACCEL, 1, [4, 5, 6]));

            let error = driver.poll().expect_err("cross-slot pair must fail");
            assert!(
                format!("{error:#}").contains("FIFO time slot changed"),
                "{error:#}"
            );
            assert!(driver.i2c.fifo.is_empty(), "corrupt FIFO was discarded");
        }

        #[test]
        fn a_time_slot_gap_restarts_the_fifo() {
            let mut driver = driver(100);
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_GYRO, 0, [1, 2, 3]));
            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_ACCEL, 0, [4, 5, 6]));
            assert_eq!(driver.poll().expect("first slot").samples.len(), 1);

            driver
                .i2c
                .fifo
                .push_back(FakeI2c::record(TAG_GYRO, 2, [7, 8, 9]));
            let error = driver.poll().expect_err("slot gap must fail");
            assert!(
                format!("{error:#}").contains("FIFO time slot jumped"),
                "{error:#}"
            );
            assert!(driver.i2c.fifo.is_empty(), "corrupt FIFO was discarded");
        }

        #[test]
        fn a_bad_tag_parity_restarts_the_fifo() {
            let mut driver = driver(100);
            let mut record = FakeI2c::record(TAG_GYRO, 0, [1, 2, 3]);
            record[0] ^= 1;
            driver.i2c.fifo.push_back(record);

            let error = driver.poll().expect_err("bad tag parity must fail");
            assert!(format!("{error:#}").contains("even-parity"), "{error:#}");
            assert!(driver.i2c.fifo.is_empty(), "corrupt FIFO was discarded");
        }

        #[test]
        fn one_hz_backlog_fits_inside_the_hard_fifo_bound() {
            let mut driver = driver(1);
            // A 1 Hz poll of the 12.5 Hz hardware rate can straddle 13 time
            // slots. Keep that accepted public rate below the hard bus budget.
            for index in 0..13 {
                let slot = (index & 0x03) as u8;
                driver
                    .i2c
                    .fifo
                    .push_back(FakeI2c::record(TAG_GYRO, slot, [1, 2, 3]));
                driver
                    .i2c
                    .fifo
                    .push_back(FakeI2c::record(TAG_ACCEL, slot, [4, 5, 6]));
            }
            assert_eq!(
                driver
                    .poll()
                    .expect("one second at the minimum hardware rate")
                    .samples
                    .len(),
                13
            );

            for _ in 0..=MAX_FIFO_RECORDS {
                driver
                    .i2c
                    .fifo
                    .push_back(FakeI2c::record(TAG_GYRO, 0, [0; 3]));
            }
            let error = driver.poll().expect_err("backlog above budget must fail");
            let expected = format!("backlog is {} records", MAX_FIFO_RECORDS + 1);
            assert!(error.to_string().contains(&expected), "{error:#}");
            assert!(driver.i2c.fifo.is_empty(), "backlogged FIFO was discarded");
        }

        #[test]
        fn an_overrun_is_bounded_and_restarts_the_fifo() {
            let mut driver = driver(100);
            driver.i2c.registers[usize::from(FIFO_STATUS1 + 1)] = FIFO_OVERRUN;
            let error = driver.poll().expect_err("overrun must fail this poll");
            assert!(error.to_string().contains("overrun=true"), "{error:#}");
            assert_eq!(driver.i2c.registers[usize::from(FIFO_CTRL4)], FIFO_STREAM);
        }

        #[test]
        fn rejects_a_different_chip_instead_of_falling_back() {
            let mut fake = FakeI2c::lsm6dso();
            fake.registers[usize::from(WHO_AM_I)] = 0x70;
            let error = Driver::initialise(fake, 0x6a, 100, |_| {})
                .err()
                .expect("wrong chip must fail");
            assert!(error.to_string().contains("expected 0x6c"), "{error:#}");
        }
    }
}

#[cfg(target_os = "linux")]
pub(crate) use linux::Sensor;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_rounds_up_to_a_common_lsm6dso_rung() {
        assert_eq!(rate(1).hz, 12.5);
        assert_eq!(rate(12).hz, 12.5);
        assert_eq!(rate(13).hz, 26.0);
        assert_eq!(rate(50).hz, 52.0);
        assert_eq!(rate(100).hz, 104.0);
        assert_eq!(rate(208).hz, 208.0);
        assert_eq!(rate(209).hz, 416.0);
        assert_eq!(rate(u16::MAX).hz, 416.0);
    }
}
