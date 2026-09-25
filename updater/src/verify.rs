//! Artifact integrity checks and bounded extraction.
//!
//! Invariant: no downloaded artifact is extracted to a live path until its SHA-256
//! matches the release manifest.

use std::fs::File;
use std::io::Read;
use std::path::Path;

use sha2::{Digest, Sha256};

use crate::Error;

/// Read buffer for streaming hash/verify. Kind to a small board while still
/// amortising syscalls.
const CHUNK: usize = 64 * 1024;

/// Bounds on what an archive may expand to.
///
/// A guard against an accidentally enormous artifact filling the eMMC. Configurable
/// because a model bundle of several ONNX policies is legitimately much larger than
/// a daemon binary (`docs/design/updater-design.md` §5.5), and a too-low ceiling would
/// reject a genuine release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArchiveLimits {
    pub max_uncompressed_bytes: u64,
    pub max_entries: usize,
}

impl Default for ArchiveLimits {
    fn default() -> Self {
        Self {
            max_uncompressed_bytes: 2 * 1024 * 1024 * 1024,
            max_entries: 50_000,
        }
    }
}

/// Check a file's SHA-256 against the manifest's hex digest.
///
/// Case-insensitive because hex casing varies between tools.
pub fn verify_sha256(path: &Path, expected_hex: &str) -> Result<(), Error> {
    let actual = sha256_hex(path)?;
    if actual.eq_ignore_ascii_case(expected_hex.trim()) {
        Ok(())
    } else {
        Err(Error::Verification(format!(
            "sha256 mismatch: manifest says {expected_hex}, artifact is {actual}"
        )))
    }
}

pub fn sha256_hex(path: &Path) -> Result<String, Error> {
    let mut file = File::open(path).map_err(|e| Error::Io {
        path: path.to_path_buf(),
        source: e,
    })?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; CHUNK];
    loop {
        let n = file.read(&mut buf).map_err(|e| Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex(&hasher.finalize()))
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write;
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// Extract a `.tar.zst` artifact into `dest`.
///
/// **Only call this after hash verification has passed.**
///
/// Path-traversal safety comes from `tar`'s own `unpack_in`, which refuses
/// absolute paths and entries that would escape the destination, rather than from
/// a hand-rolled check. On top of that we cap total uncompressed size and entry
/// count, which the library does not do — a zip bomb would otherwise fill the
/// eMMC.
pub fn extract_artifact(archive: &Path, dest: &Path, limits: ArchiveLimits) -> Result<(), Error> {
    let file = File::open(archive).map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;
    let decoder = zstd::Decoder::new(file).map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;

    std::fs::create_dir_all(dest).map_err(|e| Error::Io {
        path: dest.to_path_buf(),
        source: e,
    })?;

    let mut tar = tar::Archive::new(decoder);
    tar.set_preserve_permissions(true); // hooks and binaries need the exec bit

    let entries = tar.entries().map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;

    let mut total: u64 = 0;
    let mut count = 0usize;

    for entry in entries {
        let mut entry = entry.map_err(|e| Error::Io {
            path: archive.to_path_buf(),
            source: e,
        })?;

        count += 1;
        if count > limits.max_entries {
            // Deliberately not a verification error: this is an oversized artifact or a
            // too-tight limit, not a digest mismatch.
            return Err(Error::ArchiveTooLarge(format!(
                "archive has more than {} entries",
                limits.max_entries
            )));
        }

        total = total.saturating_add(entry.size());
        if total > limits.max_uncompressed_bytes {
            return Err(Error::ArchiveTooLarge(format!(
                "archive expands beyond {} bytes (raise max_uncompressed_bytes if this \
                 release is legitimately this big)",
                limits.max_uncompressed_bytes
            )));
        }

        let name = entry
            .path()
            .map(|p| p.display().to_string())
            .unwrap_or_default();

        // `unpack_in` returns Ok(false) when the entry path is unsafe (absolute or escaping
        // `dest`). Treat that as a verification failure, not something to skip.
        let unpacked = entry.unpack_in(dest).map_err(|e| Error::Io {
            path: dest.to_path_buf(),
            source: e,
        })?;
        if !unpacked {
            return Err(Error::Verification(format!(
                "archive entry {name:?} would escape the destination; refusing"
            )));
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_detects_change() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("f");
        std::fs::write(&path, b"hello").unwrap();
        // Known digest of "hello".
        let expected = "2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824";
        assert!(verify_sha256(&path, expected).is_ok());
        assert!(
            verify_sha256(&path, &expected.to_uppercase()).is_ok(),
            "hex casing must not matter"
        );

        std::fs::write(&path, b"hello!").unwrap();
        assert!(verify_sha256(&path, expected).is_err());
    }

    /// Build a `.tar.zst` with the given entries.
    ///
    /// The `zstd` encoder is `auto_finish`, so the frame is only completed when
    /// the builder (and thus the encoder) is dropped — hence the explicit `drop`
    /// before returning. Forgetting it yields a truncated archive.
    fn make_archive(dest: &Path, files: &[(&str, &[u8], u32)]) {
        let out = File::create(dest).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);
        for (name, body, mode) in files {
            let mut header = tar::Header::new_gnu();
            header.set_size(body.len() as u64);
            header.set_mode(*mode);
            header.set_cksum();
            builder.append_data(&mut header, name, *body).unwrap();
        }
        builder.finish().unwrap();
        drop(builder);
    }

    /// Build an archive containing a deliberately hostile entry path.
    ///
    /// `Builder::append_data` validates paths and refuses these, so the name is
    /// written straight into the header bytes to bypass it. That is the whole
    /// point: we need to prove *our extractor* rejects what a malicious producer
    /// could emit, and a well-behaved builder can't produce the input.
    fn make_archive_with_raw_name(dest: &Path, raw_name: &str, body: &[u8]) {
        let out = File::create(dest).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);

        let mut header = tar::Header::new_gnu();
        header.set_size(body.len() as u64);
        header.set_mode(0o644);
        header.set_entry_type(tar::EntryType::Regular);
        {
            let gnu = header.as_gnu_mut().unwrap();
            let bytes = raw_name.as_bytes();
            gnu.name[..bytes.len()].copy_from_slice(bytes);
        }
        header.set_cksum();

        builder.append(&header, body).unwrap();
        builder.finish().unwrap();
        drop(builder);
    }

    #[test]
    fn extracts_normal_archive() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("a.tar.zst");
        make_archive(
            &archive,
            &[
                ("bin/robotd", b"elf", 0o755),
                ("version.toml", b"v=1", 0o644),
            ],
        );

        let dest = dir.path().join("out");
        extract_artifact(&archive, &dest, ArchiveLimits::default()).unwrap();

        assert_eq!(std::fs::read(dest.join("bin/robotd")).unwrap(), b"elf");
        assert_eq!(std::fs::read(dest.join("version.toml")).unwrap(), b"v=1");
    }

    /// A traversal entry must be refused outright, not silently skipped.
    #[test]
    fn refuses_path_traversal() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("evil.tar.zst");
        make_archive_with_raw_name(&archive, "../escaped", b"pwned");

        let dest = dir.path().join("out");
        let err = extract_artifact(&archive, &dest, ArchiveLimits::default()).unwrap_err();
        assert!(matches!(err, Error::Verification(_)), "got {err:?}");
        assert!(
            !dir.path().join("escaped").exists(),
            "must not write outside dest"
        );
    }

    /// Absolute paths must not land at the filesystem root.
    #[test]
    fn refuses_absolute_path() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("abs.tar.zst");
        make_archive_with_raw_name(&archive, "/tmp/updater-should-not-exist", b"nope");

        let dest = dir.path().join("out");
        // Either refused, or stripped to a relative path inside dest — never
        // written to the absolute location.
        let _ = extract_artifact(&archive, &dest, ArchiveLimits::default());
        assert!(!Path::new("/tmp/updater-should-not-exist").exists());
    }

    #[test]
    fn refuses_too_many_entries() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("many.tar.zst");

        let out = File::create(&archive).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);
        for i in 0..32 {
            let mut header = tar::Header::new_gnu();
            header.set_size(1);
            header.set_mode(0o644);
            header.set_cksum();
            builder
                .append_data(&mut header, format!("f{i}"), &b"x"[..])
                .unwrap();
        }
        builder.finish().unwrap();
        drop(builder); // completes the zstd frame

        let dest = dir.path().join("out");
        // A tight limit for the test; the real ceiling is configurable.
        let limits = ArchiveLimits {
            max_entries: 16,
            ..ArchiveLimits::default()
        };
        let err = extract_artifact(&archive, &dest, limits).unwrap_err();
        assert!(matches!(err, Error::ArchiveTooLarge(_)), "got {err:?}");
    }

    /// The exec bit must survive extraction or post-install hooks won't run.
    #[test]
    fn preserves_executable_bit() {
        use std::os::unix::fs::PermissionsExt;

        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("x.tar.zst");
        make_archive(&archive, &[("hooks/postinstall", b"#!/bin/sh\n", 0o755)]);

        let dest = dir.path().join("out");
        extract_artifact(&archive, &dest, ArchiveLimits::default()).unwrap();

        let mode = std::fs::metadata(dest.join("hooks/postinstall"))
            .unwrap()
            .permissions()
            .mode();
        assert_ne!(mode & 0o111, 0, "exec bit lost, mode = {mode:o}");
    }
}
