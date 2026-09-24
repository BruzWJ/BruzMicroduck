//! Safe wrapper over ST's vendored VL53L5CX Ultra Lite Driver.
//!
//! Every operation is one FFI call into `vendor/vl53l5cx/shim.c`, whose surface
//! is scalars and two 64-entry arrays. The large vendor configuration and result
//! structs stay on the C side of the boundary.
//!
//! The shim stores its configuration in a file-scope static. [`Sensor::open`]
//! therefore enforces one instance per process instead of letting two handles
//! silently share and corrupt state.

#[cfg(target_os = "linux")]
use std::cell::Cell;
#[cfg(target_os = "linux")]
use std::ffi::CString;
#[cfg(target_os = "linux")]
use std::marker::PhantomData;
use std::path::Path;
#[cfg(target_os = "linux")]
use std::sync::atomic::{AtomicBool, Ordering};

#[cfg(target_os = "linux")]
use anyhow::{Context, anyhow};
use anyhow::{Result, bail};

use crate::Frame;
#[cfg(target_os = "linux")]
use crate::{COLS, ROWS, ZONES};

#[cfg(target_os = "linux")]
unsafe extern "C" {
    fn vl5_open(dev_path: *const std::ffi::c_char, addr_7bit: u8) -> i32;
    fn vl5_close();
    fn vl5_is_alive() -> i32;
    fn vl5_init() -> i32;
    fn vl5_start(freq_hz: u8) -> i32;
    fn vl5_stop() -> i32;
    fn vl5_data_ready() -> i32;
    fn vl5_get_frame(dist_mm: *mut i16, status: *mut u8) -> i32;
}

/// Held for the life of the process by the one [`Sensor`] that exists.
#[cfg(target_os = "linux")]
static TAKEN: AtomicBool = AtomicBool::new(false);

/// An open, initialised VL53L5CX, ranging or not.
#[cfg(target_os = "linux")]
pub struct Sensor {
    ranging: bool,
    // The C ULD keeps its configuration, results and transfer scratch buffer
    // in file-scope statics. `TAKEN` prevents two instances; this marker also
    // makes one instance !Sync so safe Rust cannot call it concurrently through
    // an Arc. It remains Send and may be moved to its dedicated sensor thread.
    _not_sync: PhantomData<Cell<()>>,
}

#[cfg(target_os = "linux")]
impl Sensor {
    /// Open the Qwiic device and upload the VL53L5CX firmware.
    ///
    /// The upload is roughly 90 KB over I²C and can take a few seconds at
    /// 400 kHz. It happens on `tofd`'s blocking sensor thread.
    pub fn open(bus: &Path, address: u8) -> Result<Self> {
        if TAKEN.swap(true, Ordering::AcqRel) {
            bail!("a sensor is already open in this process");
        }
        // Every failed attempt must release the claim or the daemon's retry loop
        // would turn one bus glitch into a process-lifetime failure.
        Self::open_inner(bus, address).inspect_err(|_| TAKEN.store(false, Ordering::Release))
    }

    fn open_inner(bus: &Path, address: u8) -> Result<Self> {
        let path = CString::new(bus.as_os_str().as_encoded_bytes())
            .with_context(|| format!("{} is not a usable device path", bus.display()))?;

        // SAFETY: `path` is NUL-terminated and outlives the call. The shim opens
        // the path and retains only the resulting descriptor.
        if unsafe { vl5_open(path.as_ptr(), address) } != 0 {
            bail!("cannot open {}", bus.display());
        }
        // `vl53l5cx_is_alive` verifies ST's device and revision IDs before any
        // firmware is uploaded, so another device at 0x29 is not misidentified.
        // SAFETY: the shim descriptor is open and its configuration initialised.
        if unsafe { vl5_is_alive() } != 1 {
            // SAFETY: closes the descriptor just opened; the shim is idempotent.
            unsafe { vl5_close() };
            bail!(
                "no VL53L5CX answered at {address:#04x} on {}",
                bus.display()
            );
        }

        // SAFETY: uploads firmware through the live descriptor.
        let status = unsafe { vl5_init() };
        if status != 0 {
            // SAFETY: closes the descriptor after the failed upload.
            unsafe { vl5_close() };
            bail!("VL53L5CX firmware upload failed (ULD status {status})");
        }

        Ok(Self {
            ranging: false,
            _not_sync: PhantomData,
        })
    }

    /// Start ranging at `hz`, at the fixed 8×8 resolution.
    pub fn start(&mut self, hz: u8) -> Result<()> {
        // SAFETY: the ULD is initialised and owns the open descriptor.
        let status = unsafe { vl5_start(hz) };
        if status != 0 {
            bail!("start ranging at {hz} Hz failed (ULD status {status})");
        }
        self.ranging = true;
        Ok(())
    }

    /// Is a new frame ready? `Err` means a bus failure, not merely "not yet".
    pub fn data_ready(&mut self) -> Result<bool> {
        // SAFETY: the ULD remains initialised for the life of this value.
        match unsafe { vl5_data_ready() } {
            1 => Ok(true),
            0 => Ok(false),
            _ => Err(anyhow!("the VL53L5CX stopped answering")),
        }
    }

    /// Read the 8×8 frame that the sensor has ready.
    pub fn read_frame(&mut self) -> Result<Frame> {
        let mut distance_mm = [0i16; ZONES];
        let mut status = [0u8; ZONES];
        // SAFETY: both pointers address exactly the 64 entries copied by the
        // shim, and `start` pins the hardware to 8×8 mode.
        let rc = unsafe { vl5_get_frame(distance_mm.as_mut_ptr(), status.as_mut_ptr()) };
        if rc != 0 {
            bail!("reading the VL53L5CX frame failed (ULD status {rc})");
        }
        Ok(Frame {
            rows: ROWS as u8,
            cols: COLS as u8,
            distance_mm: distance_mm.to_vec(),
            status: status.to_vec(),
        })
    }
}

#[cfg(target_os = "linux")]
impl Drop for Sensor {
    fn drop(&mut self) {
        if self.ranging {
            // SAFETY: the ULD is initialised. A failure cannot be acted on while
            // dropping; the descriptor is closed below either way.
            unsafe { vl5_stop() };
        }
        // SAFETY: closes the descriptor owned by this sole instance.
        unsafe { vl5_close() };
        TAKEN.store(false, Ordering::Release);
    }
}

/// The same API where Linux `i2c-dev` does not exist, so fake/sim builds work.
#[cfg(not(target_os = "linux"))]
pub struct Sensor(std::convert::Infallible);

#[cfg(not(target_os = "linux"))]
impl Sensor {
    pub fn open(bus: &Path, _address: u8) -> Result<Self> {
        bail!(
            "no i2c-dev on this platform, so {} cannot be opened — off a board, run `tofd --fake`",
            bus.display()
        )
    }

    pub fn start(&mut self, _hz: u8) -> Result<()> {
        match self.0 {}
    }

    pub fn data_ready(&mut self) -> Result<bool> {
        match self.0 {}
    }

    pub fn read_frame(&mut self) -> Result<Frame> {
        match self.0 {}
    }
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;

    /// A failed open must release the process-wide claim so the daemon can
    /// retry after a missing bus or transient error.
    #[test]
    fn a_failed_open_releases_the_claim() {
        let nowhere = Path::new("/dev/definitely-not-an-i2c-bus");
        assert!(Sensor::open(nowhere, 0x29).is_err());
        assert!(!TAKEN.load(Ordering::Acquire), "the claim must be released");
        assert!(
            Sensor::open(nowhere, 0x29).is_err(),
            "a retry must be possible"
        );
        assert!(!TAKEN.load(Ordering::Acquire));
    }
}
