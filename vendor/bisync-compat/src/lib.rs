//! Compatibility surface for crates that still depend on the yanked `bisync`
//! 0.3 releases. The maintained implementation is published as `bisync2`.

#![no_std]

/// Definitions used while generating the blocking variant of a driver.
pub mod synchronous {
    pub use bisync2_macros::internal_delete as only_async;
    pub use bisync2_macros::internal_noop as only_sync;
    pub use bisync2_macros::internal_strip_async as bisync;

    pub const SYNC: bool = true;
    pub const ASYNC: bool = false;
}

/// Definitions used while generating the asynchronous variant of a driver.
pub mod asynchronous {
    pub use bisync2_macros::internal_delete as only_sync;
    pub use bisync2_macros::internal_noop as only_async;
    pub use bisync2_macros::internal_noop as bisync;

    pub const SYNC: bool = false;
    pub const ASYNC: bool = true;
}
