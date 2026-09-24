//! `tofd` — owns the head ToF sensor, publishes its frames, and nothing else.
//!
//! ## Why this is its own daemon
//!
//! `architecture.md` §1 splits perception from `robotd` deliberately: a
//! perception crash must not take out motor control. This sensor makes the case
//! concretely — bringing it up uploads ~90 KB of firmware over I²C, taking
//! seconds; it shares a bus with both IMUs; and a sensor that is not fitted
//! (the common case on a duck without the head module) must be a daemon logging
//! one line, not a retry loop inside the control loop's process. Nothing in the
//! 50 Hz loop reads depth, so nothing is gained by putting it there.
//!
//! One writer owns the sensor (invariant 4), so every consumer — `robotctl
//! monitor` today, mapping and obstacle avoidance when the kinematics arrive —
//! reads the same frames from one place instead of contending for the bus.
//!
//! ## Shape
//!
//! A blocking thread drives the sensor: open, upload firmware, then poll
//! for frames and broadcast them. The socket server is async and reads no
//! hardware; a subscriber that stops reading is dropped rather than allowed to
//! slow the sensor (`broadcast` gives that for free, and a lagging consumer's gap
//! is visible in [`proto::TofFrame::seq`]).
//!
//! No frame is *stored*: this is a stream. A consumer that arrives mid-scan waits
//! for the next frame, which at 15 Hz is 66 ms away.

use std::io::ErrorKind;
use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;
use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

mod config;
mod imu;
mod status;
use imu::ImuStatus;
use status::Status;

/// Same mode and reasoning as every other socket here: the group decides who may
/// ask, and it is the same group that may watch `robot.state`.
const SOCKET_MODE: u32 = 0o660;

/// The group that may read the stream. Deliberately the same one as `robotd`'s
/// socket and `padd`'s tap: whoever may watch the robot may watch what it sees.
const GROUP: &str = "robot";

/// How many frames a slow subscriber may fall behind before it starts losing
/// them. Two seconds at 15 Hz — generous, bounded, and the loss is visible as a
/// jump in `seq` rather than a silent hole.
const FRAME_BUFFER: usize = 32;

/// How often to ask the sensor whether a frame is ready, once one is nearly due.
///
/// One 1-byte register read, so the cost is a few hundred microseconds of bus.
/// At 10 ms it adds at most that to a frame's age at 15 Hz (66 ms apart), which
/// is well inside what any consumer of depth cares about.
const POLL: Duration = Duration::from_millis(10);

/// How long before a frame is due the loop starts asking for it.
///
/// **Asking every 10 ms for the whole period is asking six times to be told no
/// once.** A frame is 66 ms away at 15 Hz and the sensor answers on its own
/// clock, so the poll only has to be running as the frame lands — the rest was
/// a hundred I²C transactions and a hundred thread wakeups a second, forever,
/// on a daemon whose sensor produces fifteen frames in that time.
///
/// Two poll intervals of margin, and the anchor is the last frame's *arrival*,
/// so the estimate can never accumulate more than one period of drift: this
/// tolerates the sensor being 20 ms early on any given frame — a 30% period
/// error, far past anything a hardware ranging timer does — and a sensor that
/// is merely late is polled for exactly as long as it was before. Frame age is
/// unchanged either way: it is still bounded by [`POLL`], because that is the
/// granularity the frame is noticed at whichever way the loop got there.
const POLL_GUARD: Duration = Duration::from_millis(20);

/// Backoff between attempts to bring a sensor up, doubling to a cap.
///
/// The two failures that matter are "not fitted" (forever, on most ducks) and
/// "the bus glitched" (transient). One backoff serves both: the transient case
/// recovers in a second, and the permanent one settles at one attempt a minute
/// instead of hammering the Qwiic bus shared with both IMUs.
const RETRY_MIN: Duration = Duration::from_secs(1);
const RETRY_MAX: Duration = Duration::from_secs(60);
/// Opening and accepting configuration is not enough to reset retry backoff:
/// a sensor can fail on its first ranging read and otherwise re-upload 90 KB
/// at the minimum interval forever.
const RETRY_RESET_AFTER: Duration = Duration::from_secs(2);
/// A ranging sensor must produce at least one frame in this floor, or three
/// configured frame periods at very low rates.
const NO_FRAME_MIN: Duration = Duration::from_secs(2);

fn no_frame_timeout(hz: u8) -> Duration {
    NO_FRAME_MIN.max(Duration::from_secs_f64(3.0 / f64::from(hz.max(1))))
}

/// The one provisioned name for the Qwiic adapter. A missing alias is a board-setup failure,
/// not a reason for the daemon to bypass the device contract through `/dev/i2c-3`.
const QWIIC_BUS: &str = "/dev/i2c-qwiic";

#[derive(Parser, Debug)]
#[command(name = "tofd", about = "Head ToF sensor daemon", version)]
struct Args {
    /// Socket to serve `tof.stream` on.
    #[arg(long, default_value = proto::socket::TOF)]
    socket: PathBuf,

    /// I²C bus device. Override only for an explicit bench setup.
    #[arg(long, default_value = QWIIC_BUS)]
    bus: PathBuf,

    /// VL53L5CX 7-bit I²C address.
    #[arg(long, default_value_t = 0x29, value_parser = parse_address)]
    address: u8,

    /// Ranging rate, Hz. 15 is what an 8×8 frame costs about 5% of a 400 kHz bus
    /// to deliver; the sensor accepts up to 15 at this resolution.
    #[arg(long, default_value_t = 15, value_parser = parse_tof_hz)]
    hz: u8,

    /// Publish a synthetic scene instead of reading hardware.
    ///
    /// For laptop development and for looking at a viewer's rendering without a
    /// sensor wired — the same reason `robotd --fake` exists. It produces all
    /// three zone classes (ranges, empty space, failed measurements), because a
    /// view that only ever sees ranges is a view whose other two cases have never
    /// been drawn.
    #[arg(long)]
    fake: bool,

    /// Head LSM6DSV16X publication rate, Hz. The sensor rate rounds up to an
    /// SFLP-supported 15/30/60/120/240/480 Hz rung.
    #[arg(long, default_value_t = 100, value_parser = parse_nonzero_hz)]
    imu_hz: u8,

    /// Head LSM6DSV16X 7-bit address. The standard board's jumper selects 0x6a.
    #[arg(
        long,
        default_value_t = qwiic_imu::HEAD_ADDRESS,
        value_parser = parse_address
    )]
    imu_address: u8,

    /// Read the head IMU for this session, whatever `[head_imu] enabled` says.
    ///
    /// The switch lives in `robotd.toml` because that is what `robotctl configure` writes; this
    /// is for trying the chip by hand on a board that has not opted in.
    #[arg(long, conflicts_with = "no_imu")]
    imu: bool,

    /// Do not read the head IMU, whatever the file says (a board without the head module, or to
    /// free the bus for a measurement).
    #[arg(long)]
    no_imu: bool,

    /// Params file. `[head_imu]` is read from it; everything else here is a flag.
    #[arg(long)]
    config: Option<PathBuf>,

    /// Read frames from a simulated body at `host:port` instead of a sensor.
    ///
    /// **The fake at the loop level, with a simulator behind it** — which is where a fake belongs
    /// here: `sensor.rs` says in as many words that the off-board `Sensor` "is not a fake sensor and
    /// must never become one", because the thing it stands for is a vendor C library talking to a
    /// bus. A frame arriving from somewhere else is a different question from a sensor that lies.
    ///
    /// The simulator answers `{"op":"tof"}` with the same 8x8 of distances and per-zone statuses
    /// this daemon publishes, so nothing downstream — `robotd`, the viewer — can tell.
    #[arg(long, conflicts_with = "fake")]
    sim: Option<String>,
}

fn parse_address(s: &str) -> Result<u8, String> {
    let (radix, digits) = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) => (16, hex),
        None => (10, s),
    };
    let address =
        u8::from_str_radix(digits, radix).map_err(|e| format!("{s:?} is not an address: {e}"))?;
    if address > 0x7f {
        return Err(format!("{s:?} is not a 7-bit I²C address"));
    }
    Ok(address)
}

fn parse_tof_hz(s: &str) -> Result<u8, String> {
    let hz = parse_nonzero_hz(s)?;
    if hz > 15 {
        return Err("VL53L5CX 8x8 ranging rate must be between 1 and 15 Hz".to_owned());
    }
    Ok(hz)
}

fn parse_nonzero_hz(s: &str) -> Result<u8, String> {
    let hz = s
        .parse::<u8>()
        .map_err(|e| format!("{s:?} is not a sample rate: {e}"))?;
    if hz == 0 {
        return Err("sample rate must be greater than zero".to_owned());
    }
    Ok(hz)
}

// One thread is plenty: the sensor is on its own std thread, and everything here
// is a socket doing nothing between frames.
#[tokio::main(flavor = "current_thread")]
async fn main() -> std::process::ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    // The shared one, not a private copy: as well as the journal line, it publishes
    // `/run/tofd/identity.json`, which is where `robotctl health` and
    // `scripts/dev-push.sh` read the release a daemon is actually running from.
    // `tofd` was the one daemon that published nothing, so both reported it as
    // silent — the exact gap the macro was written for, one daemon later.
    duck_ipc_proto::log_startup_identity!("tofd");

    let args = Args::parse();
    tracing::info!(socket = %args.socket.display(), hz = args.hz, "starting");

    let status = Arc::new(Status::new(args.hz));
    let (frames, _) = tokio::sync::broadcast::channel(FRAME_BUFFER);

    // The sensor runs on a plain thread, not a tokio task: every call into the
    // driver blocks on I²C — the firmware upload for seconds — and none of it is
    // cancellation-safe. `shutdown` lets it out of its loops at exit.
    let shutdown = Arc::new(AtomicBool::new(false));
    let sensor_thread = {
        let (status, frames, shutdown) = (status.clone(), frames.clone(), shutdown.clone());
        let bus = args.bus.clone();
        let address = args.address;
        let hz = args.hz;
        let fake = args.fake;
        let sim = args.sim.clone();
        std::thread::Builder::new()
            .name("tof-sensor".to_owned())
            .spawn(move || {
                if let Some(addr) = sim {
                    sim_loop(&addr, hz, &status, &frames, &shutdown);
                } else if fake {
                    fake_loop(hz, &status, &frames, &shutdown);
                } else {
                    sensor_loop(&bus, address, hz, &status, &frames, &shutdown);
                }
            })
            .expect("spawn the sensor thread")
    };

    // The head IMU on its own thread and channel, on the same bus. Its socket is the same one;
    // subscribers pick the stream by method.
    //
    // **Off unless `[head_imu] enabled` says otherwise**, and that default is the measurement in
    // `docs/project/tof-on-demand.md`: nothing subscribes to the stream yet, and a duck that is
    // not mapping should not pay for an otherwise unused high-rate sensor from boot.
    //
    // Skipped for --sim/--fake too: there is no real bus behind either.
    let imu_status = Arc::new(ImuStatus::new(args.imu_hz));
    let (imu_frames, _) = tokio::sync::broadcast::channel(imu::FRAME_BUFFER);
    let config_path = args.config.clone().unwrap_or_else(config::default_path);
    let configured = config::load(&config_path, args.config.is_some())
        .head_imu
        .enabled;
    let wanted = args.imu || (configured && !args.no_imu);
    let imu_thread = if !wanted || args.fake || args.sim.is_some() {
        // Said out loud, and said by the stream too: a subscriber gets this sentence instead of
        // frames, because "no samples" and "no LSM6DSV16X fitted" are different answers and only one
        // of them is somebody's mistake.
        if !wanted {
            tracing::info!(
                config = %config_path.display(),
                "the head IMU is off; set [head_imu] enabled = true to read it"
            );
            imu_status.off();
        } else if args.fake {
            imu_status.unavailable("the fake depth backend does not emulate a head IMU");
        } else {
            imu_status.unavailable("the simulator depth backend does not publish a head IMU");
        }
        None
    } else {
        let (imu_status, imu_frames, shutdown) =
            (imu_status.clone(), imu_frames.clone(), shutdown.clone());
        let bus = args.bus.clone();
        let hz = args.imu_hz;
        let address = args.imu_address;
        Some(
            std::thread::Builder::new()
                .name("head-imu".to_owned())
                .spawn(move || {
                    imu::imu_loop(&bus, address, hz, &imu_status, &imu_frames, &shutdown)
                })
                .expect("spawn the head-imu thread"),
        )
    };

    let served = serve(&args.socket, &status, &frames, &imu_status, &imu_frames).await;
    shutdown.store(true, Ordering::Release);
    let _ = sensor_thread.join();
    if let Some(t) = imu_thread {
        let _ = t.join();
    }

    match served {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "tofd is stopping");
            std::process::ExitCode::FAILURE
        }
    }
}

/// How long after one frame the sensor cannot yet have the next, at `hz`.
///
/// Saturating rather than clamped by hand: a rate whose period is shorter than
/// [`POLL_GUARD`] leaves nothing to skip, and the loop then polls straight
/// through exactly as it did before there was a guard.
fn quiet_period(hz: u8) -> Duration {
    Duration::from_secs_f64(1.0 / f64::from(hz.max(1))).saturating_sub(POLL_GUARD)
}

/// Bring the sensor up and stream from it, forever, with a backoff between
/// attempts. Never returns until shutdown.
fn sensor_loop(
    bus: &Path,
    address: u8,
    hz: u8,
    status: &Arc<Status>,
    frames: &tokio::sync::broadcast::Sender<proto::TofFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    let started = Instant::now();
    let mut seq = 0u64;
    let mut backoff = RETRY_MIN;
    let mut said = false;
    let quiet = quiet_period(hz);

    while !shutdown.load(Ordering::Acquire) {
        match open_sensor(bus, address, hz) {
            Ok(mut sensor) => {
                tracing::warn!(sensor = tof::SENSOR_NAME, hz, "ranging");
                status.down("sensor initialised; waiting for its first frame");
                let opened_at = Instant::now();
                let mut last_frame_at = opened_at;
                let frame_timeout = no_frame_timeout(hz);
                let mut announced = false;
                let mut retry_reset = false;

                // When the sensor could not possibly have a frame yet, so there
                // is nothing to ask it until then — see [`POLL_GUARD`]. `None`
                // before the first frame and after any poll that came up empty,
                // both of which mean "as far as this loop knows, one is due
                // now": the wait is only ever skipped forward by a frame that
                // actually arrived.
                let mut quiet_until: Option<Instant> = None;

                // Stream until the sensor stops answering, then fall through to
                // the backoff and try the whole bring-up again.
                while !shutdown.load(Ordering::Acquire) {
                    if let Some(until) = quiet_until.take() {
                        // Through the sliced sleep rather than `thread::sleep`,
                        // for the reason that helper exists: at 15 Hz this is
                        // 47 ms and either would do, but `--hz 1` makes it most
                        // of a second, and exit must not wait it out.
                        sleep_unless_shutdown(
                            until.saturating_duration_since(Instant::now()),
                            shutdown,
                        );
                    }
                    match sensor.data_ready() {
                        Ok(true) => match sensor.read_frame() {
                            Ok(frame) => {
                                let now = Instant::now();
                                last_frame_at = now;
                                quiet_until = Some(now + quiet);
                                if !announced {
                                    status.up(tof::SENSOR_NAME);
                                    said = false;
                                    announced = true;
                                }
                                if !retry_reset && opened_at.elapsed() >= RETRY_RESET_AFTER {
                                    backoff = RETRY_MIN;
                                    retry_reset = true;
                                }
                                seq += 1;
                                // No subscribers is the normal state — nobody is
                                // watching most of the time — so a send that
                                // finds none is not a failure.
                                let _ = frames.send(proto::TofFrame {
                                    seq,
                                    at_us: started.elapsed().as_micros() as u64,
                                    t_ns: proto::clock::monotonic_ns(),
                                    rows: frame.rows,
                                    cols: frame.cols,
                                    distance_mm: frame.distance_mm,
                                    status: frame.status,
                                });
                            }
                            Err(e) => {
                                tracing::warn!(error = %e, "lost the sensor mid-frame");
                                break;
                            }
                        },
                        Ok(false) => {
                            if last_frame_at.elapsed() >= frame_timeout {
                                tracing::warn!(
                                    silent_ms = last_frame_at.elapsed().as_millis(),
                                    "VL53L5CX stopped producing frames; reopening"
                                );
                                break;
                            }
                            std::thread::sleep(POLL);
                        }
                        Err(e) => {
                            tracing::warn!(error = %e, "lost the sensor");
                            break;
                        }
                    }
                }
                status.down("the sensor stopped answering; retrying");
            }
            Err(e) => {
                // Said once per run of failures, not once per attempt: a duck
                // with no ToF fitted would otherwise write this line into the
                // journal forever.
                if !said {
                    said = true;
                    tracing::warn!(error = %e, "no ToF sensor; retrying in the background");
                }
                status.down(&e.to_string());
            }
        }
        sleep_unless_shutdown(backoff, shutdown);
        backoff = (backoff * 2).min(RETRY_MAX);
    }
}

/// A synthetic scene at the configured rate: a wall receding across the frame, a
/// near object, a column of empty space and one of failed measurements.
///
/// Deliberately not a flat gradient. The three zone classes render differently
/// and the two non-range ones are the easy ones to get wrong, so `--fake` shows
/// all three from the first frame.
fn fake_loop(
    hz: u8,
    status: &Arc<Status>,
    frames: &tokio::sync::broadcast::Sender<proto::TofFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    let started = Instant::now();
    let period = Duration::from_secs_f64(1.0 / f64::from(hz.max(1)));
    status.up("fake");
    let mut seq = 0u64;

    while !shutdown.load(Ordering::Acquire) {
        seq += 1;
        let mut distance_mm = vec![0i16; tof::ZONES];
        let mut zone_status = vec![tof::STATUS_NO_TARGET; tof::ZONES];
        // A slow sweep, so a viewer shows something moving rather than a still.
        let phase = started.elapsed().as_secs_f32() * 0.5;

        for row in 0..tof::ROWS {
            for col in 0..tof::COLS {
                let i = row * tof::COLS + col;
                match col {
                    // One column the sensor could not measure, and one it measured
                    // as empty: the two cases a distance-only view cannot tell
                    // apart.
                    2 => zone_status[i] = 4,
                    5 => zone_status[i] = tof::STATUS_NO_TARGET,
                    _ => {
                        let sweep = (phase + row as f32 * 0.4).sin() * 0.5 + 0.5;
                        let metres = 0.15 + 3.0 * sweep * (col as f32 + 1.0) / tof::COLS as f32;
                        distance_mm[i] = (metres * 1000.0) as i16;
                        zone_status[i] = 5;
                    }
                }
            }
        }

        let _ = frames.send(proto::TofFrame {
            seq,
            at_us: started.elapsed().as_micros() as u64,
            t_ns: proto::clock::monotonic_ns(),
            rows: tof::ROWS as u8,
            cols: tof::COLS as u8,
            distance_mm,
            status: zone_status,
        });
        sleep_unless_shutdown(period, shutdown);
    }
}

/// Frames from a simulated body, at the sensor's own rate.
///
/// Newline-delimited JSON over TCP, the same link `duck_control::sim` uses for the servo bus — one
/// handshake, then a request per frame. A simulator that goes away is one missed frame and a
/// reconnect, not a dead daemon: MuJoCo is restarted whenever the number of ducks changes, and a
/// duck is expected to live through that.
fn sim_loop(
    addr: &str,
    hz: u8,
    status: &Arc<Status>,
    frames: &tokio::sync::broadcast::Sender<proto::TofFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    use std::io::{BufRead, Write};

    let started = Instant::now();
    let period = Duration::from_secs_f64(1.0 / f64::from(hz.max(1)));
    let mut seq = 0u64;
    let mut link: Option<SimLink> = None;
    let mut complained = false;

    while !shutdown.load(Ordering::Acquire) {
        if link.is_none() {
            match connect_sim(addr) {
                Ok(fresh) => {
                    tracing::info!(%addr, "simulated depth");
                    status.up("sim");
                    complained = false;
                    link = Some(fresh);
                }
                Err(e) => {
                    if !complained {
                        complained = true;
                        tracing::warn!(%addr, error = %e, "no simulated body; retrying");
                        status.down(&format!("no simulated body at {addr}"));
                    }
                    sleep_unless_shutdown(period, shutdown);
                    continue;
                }
            }
        }

        let (write, read) = link.as_mut().expect("just connected");
        let frame = write.write_all(b"{\"op\":\"tof\"}\n").and_then(|()| {
            let mut line = String::new();
            read.read_line(&mut line)?;
            if line.is_empty() {
                return Err(std::io::Error::other("the simulator closed the connection"));
            }
            serde_json::from_str::<SimDepth>(&line).map_err(std::io::Error::other)
        });

        match frame {
            Ok(depth) => {
                seq += 1;
                let _ = frames.send(proto::TofFrame {
                    seq,
                    at_us: started.elapsed().as_micros() as u64,
                    t_ns: proto::clock::monotonic_ns(),
                    rows: depth.rows,
                    cols: depth.cols,
                    distance_mm: depth.distance_mm,
                    status: depth.status,
                });
            }
            Err(e) => {
                tracing::debug!(error = %e, "lost the simulated body");
                link = None;
                continue;
            }
        }
        sleep_unless_shutdown(period, shutdown);
    }
}

type SimLink = (std::net::TcpStream, std::io::BufReader<std::net::TcpStream>);

/// One connection to a simulated body, handshake included.
fn connect_sim(addr: &str) -> std::io::Result<SimLink> {
    use std::io::{BufRead, Write};

    let stream = std::net::TcpStream::connect(addr)?;
    // Nagle would add tens of milliseconds to a 15 Hz request/response, which is most of a frame.
    let _ = stream.set_nodelay(true);
    let _ = stream.set_read_timeout(Some(Duration::from_millis(500)));
    let mut write = stream.try_clone()?;
    let mut read = std::io::BufReader::new(stream);
    write.write_all(b"{\"op\":\"hello\",\"protocol\":1,\"joints\":15}\n")?;
    let mut line = String::new();
    read.read_line(&mut line)?;
    if line.is_empty() {
        return Err(std::io::Error::other(
            "the simulator hung up during the handshake",
        ));
    }
    Ok((write, read))
}

/// What the simulator answers `{"op":"tof"}` with.
#[derive(serde::Deserialize)]
struct SimDepth {
    rows: u8,
    cols: u8,
    distance_mm: Vec<i16>,
    status: Vec<u8>,
}

/// Open the ToF on the one configured Qwiic adapter and start ranging.
fn open_sensor(bus: &Path, address: u8, hz: u8) -> Result<tof::Sensor> {
    let mut sensor = tof::Sensor::open(bus, address)?;
    tracing::info!(bus = %bus.display(), address = format!("{address:#04x}"), "VL53L5CX found");
    sensor.start(hz)?;
    Ok(sensor)
}

fn sleep_unless_shutdown(total: Duration, shutdown: &Arc<AtomicBool>) {
    // Sliced so exit does not wait out a minute of backoff.
    const SLICE: Duration = Duration::from_millis(100);
    let deadline = Instant::now() + total;
    while Instant::now() < deadline {
        if shutdown.load(Ordering::Acquire) {
            return;
        }
        std::thread::sleep(SLICE.min(deadline.saturating_duration_since(Instant::now())));
    }
}

/// Accept subscribers until a signal says to stop.
async fn serve(
    socket: &Path,
    status: &Arc<Status>,
    frames: &tokio::sync::broadcast::Sender<proto::TofFrame>,
    imu_status: &Arc<ImuStatus>,
    imu_frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
) -> Result<()> {
    if let Some(parent) = socket.parent() {
        // `RuntimeDirectory=tofd` has already made this on a board; tried anyway
        // for a `tofd` run by hand.
        let _ = std::fs::create_dir_all(parent);
    }
    // systemd removes the runtime directory when the unit stops, so a stale
    // socket means a `tofd` killed outside its unit. Removing it beats refusing
    // to start over a file whose owner is gone.
    if socket.exists() {
        let _ = std::fs::remove_file(socket);
    }

    let listener =
        UnixListener::bind(socket).with_context(|| format!("binding {}", socket.display()))?;
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))?;
    if let Err(e) = give_to_group(socket, GROUP) {
        // Not fatal, and said out loud with what it means: the socket exists, and
        // only `tofd` and root can read it. On a board that is a broken install;
        // on a laptop it is a machine with no `robot` group, which is ordinary.
        tracing::warn!(
            error = %e, group = GROUP, socket = %socket.display(),
            "the depth stream stays private to tofd — nothing else can read it"
        );
    }
    tracing::info!(
        path = %socket.display(),
        mode = format!("{SOCKET_MODE:o}"),
        "serving tof.stream"
    );

    let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    let mut int = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())?;

    loop {
        tokio::select! {
            accepted = listener.accept() => match accepted {
                Ok((stream, _)) => {
                    let status = status.clone();
                    let imu_status = imu_status.clone();
                    let frames = frames.clone();
                    let imu_frames = imu_frames.clone();
                    tokio::spawn(async move {
                        if let Err(e) = subscriber(stream, &status, &frames, &imu_status, &imu_frames).await {
                            tracing::debug!(error = %e, "subscriber ended");
                        }
                    });
                }
                Err(e) => tracing::warn!(error = %e, "accept failed"),
            },
            _ = term.recv() => {
                tracing::warn!("SIGTERM; stopping");
                return Ok(());
            }
            _ = int.recv() => {
                tracing::warn!("SIGINT; stopping");
                return Ok(());
            }
        }
    }
}

/// One subscriber: its request, then frames until it goes away.
/// Takes the two `Sender`s and subscribes only in the arm that matched, so a receiver exists
/// where the method is known and nowhere else. Subscribing on `accept` made `receiver_count()`
/// count connections rather than interest: a client asking for `head_imu.stream` held a depth
/// receiver, and one that connected and said nothing held both.
async fn subscriber(
    stream: UnixStream,
    status: &Arc<Status>,
    frames: &tokio::sync::broadcast::Sender<proto::TofFrame>,
    imu_status: &Arc<ImuStatus>,
    imu_frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
) -> Result<()> {
    let (read, mut write) = stream.into_split();
    let mut reader = BufReader::new(read);
    let mut line = String::new();

    // One request, and it must be `tof.stream`. Anything else is answered and the
    // connection kept, so a client that spells a method wrong is told rather than
    // dropped.
    loop {
        line.clear();
        if reader.read_line(&mut line).await? == 0 {
            return Ok(());
        }
        let request: proto::Request = match serde_json::from_str(line.trim()) {
            Ok(request) => request,
            Err(e) => {
                let response = proto::Response::err(
                    None,
                    proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
                );
                write_line(&mut write, &response).await?;
                continue;
            }
        };
        let id = request.id.clone();
        match request.as_call() {
            Ok(proto::Call::TofStream) => {
                let response = proto::Response::ok(id, &status.result());
                write_line(&mut write, &response).await?;
                let mut frames = frames.subscribe();
                return stream_tof(&mut write, &mut frames).await;
            }
            Ok(proto::Call::HeadImuStream) => {
                let response = proto::Response::ok(id, &imu_status.result());
                write_line(&mut write, &response).await?;
                let mut imu_frames = imu_frames.subscribe();
                return stream_imu(&mut write, &mut imu_frames).await;
            }
            _ => {
                let response = proto::Response::err(
                    id,
                    proto::Error::new(
                        proto::code::METHOD_NOT_FOUND,
                        "tofd serves tof.stream and head_imu.stream and nothing else",
                    ),
                );
                write_line(&mut write, &response).await?;
            }
        }
    }
}

/// Stream ToF frames as notifications until the socket closes or the consumer lags out. A lag is
/// not fatal — the gap shows in `seq`, and the next frame is 66 ms away.
async fn stream_tof(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    frames: &mut tokio::sync::broadcast::Receiver<proto::TofFrame>,
) -> Result<()> {
    loop {
        match frames.recv().await {
            Ok(frame) => {
                let notification = proto::Request::notify_tof_frame(&frame);
                if let Err(e) = write_line(write, &notification).await {
                    return if e.kind() == ErrorKind::BrokenPipe {
                        Ok(())
                    } else {
                        Err(e.into())
                    };
                }
            }
            Err(tokio::sync::broadcast::error::RecvError::Lagged(missed)) => {
                tracing::debug!(missed, "a tof subscriber fell behind");
            }
            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
        }
    }
}

/// Stream head-IMU samples as notifications; same lag/broken-pipe handling as the ToF stream.
async fn stream_imu(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    frames: &mut tokio::sync::broadcast::Receiver<proto::HeadImuFrame>,
) -> Result<()> {
    loop {
        match frames.recv().await {
            Ok(frame) => {
                let notification = proto::Request::notify_head_imu_frame(&frame);
                if let Err(e) = write_line(write, &notification).await {
                    return if e.kind() == ErrorKind::BrokenPipe {
                        Ok(())
                    } else {
                        Err(e.into())
                    };
                }
            }
            Err(tokio::sync::broadcast::error::RecvError::Lagged(missed)) => {
                tracing::debug!(missed, "an imu subscriber fell behind");
            }
            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
        }
    }
}

/// Hand the socket to `GROUP`, so the same people who may watch `robot.state` may
/// watch this. Mirrors `padd`'s tap, including that a missing group is a warning.
fn give_to_group(socket: &Path, group: &str) -> std::io::Result<()> {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;

    let name = CString::new(group).map_err(std::io::Error::other)?;
    // SAFETY: `getgrnam` reads the group database and returns a pointer into
    // storage it owns. The name is a valid C string for the length of the call,
    // and nothing else in this process calls into the group database.
    let entry = unsafe { libc::getgrnam(name.as_ptr()) };
    if entry.is_null() {
        return Err(std::io::Error::other(format!(
            "no {group} group on this system"
        )));
    }
    // SAFETY: checked non-null immediately above, and `struct group` is fully
    // initialised by `getgrnam` when it returns a pointer at all.
    let gid = unsafe { (*entry).gr_gid };

    let path = CString::new(socket.as_os_str().as_bytes()).map_err(std::io::Error::other)?;
    // SAFETY: a valid C string path; `-1` for the owner is the documented
    // "leave it alone".
    if unsafe { libc::chown(path.as_ptr(), u32::MAX, gid) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

async fn write_line(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    message: &impl serde::Serialize,
) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)?;
    line.push(b'\n');
    write.write_all(&line).await
}

#[cfg(test)]
mod tests {
    use super::*;

    /// **A connection that has asked for nothing wants nothing**, and one that asked for depth
    /// does not want the IMU. `accept` used to subscribe to both channels before reading a byte
    /// of the request, so `receiver_count()` counted open connections rather than interest —
    /// which is the number anything gating a sensor on demand would have to trust.
    ///
    /// Drives the real `subscriber` over a socket pair and counts receivers on the senders it
    /// holds, since that count is the thing that was wrong.
    #[tokio::test]
    async fn a_connection_subscribes_only_to_what_it_asked_for() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("tofd.sock");
        let listener = UnixListener::bind(&path).unwrap();

        let (frames, _) = tokio::sync::broadcast::channel::<proto::TofFrame>(4);
        let (imu_frames, _) = tokio::sync::broadcast::channel::<proto::HeadImuFrame>(4);
        let status = Arc::new(Status::new(15));
        let imu_status = Arc::new(ImuStatus::new(100));

        let served = {
            let (frames, imu_frames) = (frames.clone(), imu_frames.clone());
            tokio::spawn(async move {
                let (stream, _) = listener.accept().await.unwrap();
                let _ = subscriber(stream, &status, &frames, &imu_status, &imu_frames).await;
            })
        };

        let client = tokio::net::UnixStream::connect(&path).await.unwrap();
        let (read, mut write) = client.into_split();
        let mut reader = BufReader::new(read);

        // Connected, and nothing asked for yet: neither channel has a receiver.
        assert_eq!(
            frames.receiver_count(),
            0,
            "a silent connection wants no depth"
        );
        assert_eq!(imu_frames.receiver_count(), 0, "nor the imu");

        // Ask for depth only.
        let request = format!(
            "{}\n",
            serde_json::to_string(&proto::Request::call(
                proto::Id::Number(1),
                &proto::Call::TofStream
            ))
            .unwrap()
        );
        write.write_all(request.as_bytes()).await.unwrap();
        let mut answer = String::new();
        reader.read_line(&mut answer).await.unwrap();
        assert!(answer.contains("result"), "{answer}");

        // The answer is written before the subscribe, so give the task its next poll.
        for _ in 0..100 {
            if frames.receiver_count() == 1 {
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(2)).await;
        }
        assert_eq!(frames.receiver_count(), 1, "depth was asked for");
        assert_eq!(
            imu_frames.receiver_count(),
            0,
            "a client that asked for depth must not hold an imu receiver"
        );

        // `stream_tof` is parked on the channel, not on the socket, so it would sit there until a
        // frame arrived. Nothing here sends one, and the counts have already been taken.
        served.abort();
    }

    #[test]
    fn addresses_parse_in_both_bases() {
        assert_eq!(parse_address("0x29"), Ok(0x29));
        assert_eq!(parse_address("41"), Ok(41));
        assert!(parse_address("0x1ff").is_err(), "wider than an address");
        assert!(parse_address("0x80").is_err(), "not a 7-bit address");
        assert!(parse_address("nope").is_err());
    }

    #[test]
    fn command_line_rates_match_the_hardware() {
        assert_eq!(parse_tof_hz("1"), Ok(1));
        assert_eq!(parse_tof_hz("15"), Ok(15));
        assert!(parse_tof_hz("0").is_err());
        assert!(parse_tof_hz("16").is_err());
        assert_eq!(parse_nonzero_hz("100"), Ok(100));
        assert!(parse_nonzero_hz("0").is_err());
    }

    /// The backoff must climb and stop climbing — a duck with no sensor fitted
    /// spends its whole life in this loop.
    #[test]
    fn the_backoff_is_capped() {
        let mut backoff = RETRY_MIN;
        for _ in 0..20 {
            backoff = (backoff * 2).min(RETRY_MAX);
        }
        assert_eq!(backoff, RETRY_MAX);
        assert!(RETRY_MIN < RETRY_MAX);
    }

    /// **The saving, as arithmetic rather than as a claim in a comment.**
    ///
    /// At the shipped rate a frame used to cost seven `data_ready` reads to
    /// find, six of them answered no. Widening [`POLL_GUARD`] until the quiet
    /// stretch disappears would leave the loop correct and the reason for it
    /// gone, which is exactly the change nothing else here would notice.
    #[test]
    fn a_frame_at_the_shipped_rate_costs_three_polls_instead_of_seven() {
        let period = Duration::from_secs_f64(1.0 / 15.0);
        let polls = |window: Duration| (window.as_secs_f64() / POLL.as_secs_f64()).ceil() as u32;

        assert_eq!(polls(period), 7, "what polling the whole period cost");
        // The guard's window, plus the poll that finds the frame at the end of it.
        assert_eq!(polls(period - quiet_period(15)) + 1, 3);
    }

    /// The anchor is the last frame's arrival, so the guard only ever has to
    /// absorb one period of drift. 20 ms of it at 15 Hz is a 30% period error —
    /// far past anything a hardware ranging timer does.
    #[test]
    fn the_guard_tolerates_a_sensor_running_early() {
        let period = Duration::from_secs_f64(1.0 / 15.0);
        let slack = POLL_GUARD.as_secs_f64() / period.as_secs_f64();
        assert!(slack > 0.25, "only {slack:.0}% of a period of margin");
    }

    /// A rate whose period is shorter than the guard has nothing to skip, and
    /// must poll straight through rather than underflow into a long sleep.
    #[test]
    fn a_rate_faster_than_the_guard_polls_straight_through() {
        assert_eq!(quiet_period(60), Duration::ZERO);
        assert_eq!(quiet_period(u8::MAX), Duration::ZERO);
    }

    /// The CLI rejects zero, and this helper remains defensive if another caller
    /// is added later.
    #[test]
    fn a_zero_rate_is_treated_as_one_hertz() {
        assert_eq!(quiet_period(0), Duration::from_secs(1) - POLL_GUARD);
    }

    #[test]
    fn a_silent_ranging_sensor_has_a_finite_rate_aware_deadline() {
        assert_eq!(no_frame_timeout(15), NO_FRAME_MIN);
        assert_eq!(no_frame_timeout(1), Duration::from_secs(3));
        assert_eq!(no_frame_timeout(0), Duration::from_secs(3));
    }
}
