//! Release compatibility for the one supported electronics platform.
//!
//! This replica has one hardware target: Radxa Zero 3W, the Qwiic sensor chain and an
//! OpenRB-150. Releases carry its numeric revision so an updater can reject an artifact built
//! for incompatible future hardware without making the target a per-robot setting.

/// Hardware revision written into every release manifest and checked by every updater.
pub const HW_REV: u32 = 1;
