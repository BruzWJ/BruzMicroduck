//! Local-directory source.
//!
//! Two jobs:
//!  - CI tests drive the **real** engine code path with no network, so tests can't
//!    drift from production behaviour (`docs/design/updater-design.md` §16.1);
//!  - the dev sideload flow, where a locally-built artifact is applied without
//!    publishing it first (§15).
//!
//! Layout expected under `path`:
//! ```text
//!   <version>.manifest.json           e.g. 1.4.2.manifest.json
//!   <whatever the manifest's `url` names>
//! ```

use std::path::{Path, PathBuf};

use crate::Error;
use crate::manifest::Manifest;
use crate::source::{FetchedArtifact, FetchedManifest, ProgressSink, Source};

const MANIFEST_SUFFIX: &str = ".manifest.json";

pub struct LocalDir {
    root: PathBuf,
}

impl LocalDir {
    pub fn new(root: PathBuf) -> Self {
        Self { root }
    }

    fn manifest_path(&self, version: &semver::Version) -> PathBuf {
        self.root.join(format!("{version}{MANIFEST_SUFFIX}"))
    }

    /// Versions present, newest first. Entries whose name isn't a version are
    /// ignored rather than treated as errors — the directory also holds artifacts.
    fn versions(&self) -> Result<Vec<semver::Version>, Error> {
        let entries = std::fs::read_dir(&self.root).map_err(|e| Error::Io {
            path: self.root.clone(),
            source: e,
        })?;

        let mut versions: Vec<_> = entries
            .filter_map(Result::ok)
            .filter_map(|e| {
                let name = e.file_name().to_str()?.to_owned();
                let stem = name.strip_suffix(MANIFEST_SUFFIX)?;
                semver::Version::parse(stem).ok()
            })
            .collect();
        versions.sort_by(|a, b| b.cmp(a));
        Ok(versions)
    }

    fn read_manifest(&self, path: &Path) -> Result<FetchedManifest, Error> {
        let bytes = std::fs::read(path).map_err(|e| Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

        let parsed: Manifest = serde_json::from_slice(&bytes)
            .map_err(|e| Error::Corrupt(format!("{}: {e}", path.display())))?;

        Ok(FetchedManifest { bytes, parsed })
    }

    /// Artifact path from a manifest `url`.
    ///
    /// Only a bare filename is accepted: a manifest must not be able to point the
    /// engine at an arbitrary path on the robot.
    fn artifact_path(&self, manifest: &Manifest) -> Result<PathBuf, Error> {
        let name = Path::new(&manifest.url);
        let is_plain_filename = name.components().count() == 1 && name.file_name().is_some();
        if !is_plain_filename {
            return Err(Error::Verification(format!(
                "local manifest url must be a bare filename, got {:?}",
                manifest.url
            )));
        }
        Ok(self.root.join(name))
    }
}

#[async_trait::async_trait]
impl Source for LocalDir {
    async fn latest_manifest(&self) -> Result<FetchedManifest, Error> {
        let newest =
            self.versions()?.into_iter().next().ok_or_else(|| {
                Error::Network(format!("no manifests in {}", self.root.display()))
            })?;
        self.read_manifest(&self.manifest_path(&newest))
    }

    async fn manifest_for(&self, version: &semver::Version) -> Result<FetchedManifest, Error> {
        let path = self.manifest_path(version);
        if !path.exists() {
            return Err(Error::Network(format!(
                "no manifest for version {version} in {}",
                self.root.display()
            )));
        }
        self.read_manifest(&path)
    }

    /// A ref names a manifest directly: `my-branch` → `my-branch.manifest.json`.
    ///
    /// Supported here, not just on the GitHub source, for two reasons: it makes the whole
    /// `--ref` path testable through the real engine with no network, and it is the offline
    /// sideload story — copy a directory onto a board and install by name.
    ///
    /// **The ref becomes a filename, so it is validated.** Unlike the GitHub source, where
    /// `feature/foo` is a legitimate tag component, a separator here would escape the
    /// directory — `../../etc/anything` is a path, not a branch. Rejected rather than
    /// sanitised, because silently rewriting a caller's ref would install something other
    /// than what they named.
    async fn manifest_at_ref(&self, git_ref: &str) -> Result<FetchedManifest, Error> {
        if git_ref.is_empty()
            || git_ref.contains('/')
            || git_ref.contains('\\')
            || git_ref.contains("..")
        {
            return Err(Error::Verification(format!(
                "refusing ref {git_ref:?}: a local_dir ref becomes a filename, so it may not \
                 contain a path separator or `..`"
            )));
        }

        let path = self.root.join(format!("{git_ref}{MANIFEST_SUFFIX}"));
        if !path.exists() {
            return Err(Error::Network(format!(
                "no manifest for ref {git_ref:?} in {}",
                self.root.display()
            )));
        }
        self.read_manifest(&path)
    }

    async fn fetch_artifact(
        &self,
        manifest: &Manifest,
        dest_dir: &Path,
        progress: ProgressSink,
    ) -> Result<FetchedArtifact, Error> {
        let src = self.artifact_path(manifest)?;

        std::fs::create_dir_all(dest_dir).map_err(|e| Error::Io {
            path: dest_dir.to_path_buf(),
            source: e,
        })?;

        let file_name = src
            .file_name()
            .ok_or_else(|| Error::Corrupt("artifact path has no file name".into()))?;
        let artifact = dest_dir.join(file_name);

        // Copy rather than symlink, so staging behaves identically to a real
        // download and the caller can safely delete the staging tree.
        let bytes = std::fs::copy(&src, &artifact).map_err(|e| Error::Io {
            path: src.clone(),
            source: e,
        })?;
        // Local copies are instant, but emit the terminal progress so subscribers
        // see the same phase sequence they would for a network fetch.
        let _ = progress.send((bytes, Some(bytes)));

        Ok(FetchedArtifact { artifact })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_manifest(root: &Path, version: &str, url: &str) {
        let manifest = serde_json::json!({
            "channel": "daemon",
            "version": version,
            "url": url,
            "sha256": "00".repeat(32),
        });
        let path = root.join(format!("{version}{MANIFEST_SUFFIX}"));
        std::fs::write(&path, serde_json::to_vec(&manifest).unwrap()).unwrap();
    }

    #[tokio::test]
    async fn latest_picks_highest_semver_not_lexical() {
        let dir = tempfile::tempdir().unwrap();
        // Lexically "1.9.0" > "1.10.0"; semver disagrees. This is the bug plain
        // string sorting would introduce.
        for v in ["1.9.0", "1.10.0", "1.2.0"] {
            write_manifest(dir.path(), v, "a.tar.zst");
        }

        let source = LocalDir::new(dir.path().to_path_buf());
        let fetched = source.latest_manifest().await.unwrap();
        assert_eq!(fetched.parsed.version, semver::Version::new(1, 10, 0));
    }

    #[tokio::test]
    async fn manifest_for_exact_version() {
        let dir = tempfile::tempdir().unwrap();
        write_manifest(dir.path(), "1.0.0", "a.tar.zst");
        write_manifest(dir.path(), "2.0.0", "a.tar.zst");

        let source = LocalDir::new(dir.path().to_path_buf());
        let fetched = source
            .manifest_for(&semver::Version::new(1, 0, 0))
            .await
            .unwrap();
        assert_eq!(fetched.parsed.version, semver::Version::new(1, 0, 0));
    }

    #[tokio::test]
    async fn missing_version_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let source = LocalDir::new(dir.path().to_path_buf());
        assert!(
            source
                .manifest_for(&semver::Version::new(9, 9, 9))
                .await
                .is_err()
        );
    }

    /// The raw bytes must come back untouched because the engine embeds the exact source
    /// manifest in the installed release.
    #[tokio::test]
    async fn returns_bytes_as_read() {
        let dir = tempfile::tempdir().unwrap();
        write_manifest(dir.path(), "1.0.0", "a.tar.zst");
        let on_disk = std::fs::read(dir.path().join("1.0.0.manifest.json")).unwrap();

        let source = LocalDir::new(dir.path().to_path_buf());
        let fetched = source.latest_manifest().await.unwrap();
        assert_eq!(fetched.bytes, on_disk);
    }

    /// A manifest must not be able to make the engine read an arbitrary path.
    #[tokio::test]
    async fn refuses_path_in_manifest_url() {
        let dir = tempfile::tempdir().unwrap();
        write_manifest(dir.path(), "1.0.0", "../../etc/passwd");

        let source = LocalDir::new(dir.path().to_path_buf());
        let fetched = source.latest_manifest().await.unwrap();
        let (tx, _rx) = tokio::sync::mpsc::unbounded_channel();
        let err = source
            .fetch_artifact(&fetched.parsed, dir.path(), tx)
            .await
            .unwrap_err();
        assert!(matches!(err, Error::Verification(_)), "got {err:?}");
    }

    #[tokio::test]
    async fn fetch_copies_artifact() {
        let dir = tempfile::tempdir().unwrap();
        write_manifest(dir.path(), "1.0.0", "payload.tar.zst");
        std::fs::write(dir.path().join("payload.tar.zst"), b"payload-bytes").unwrap();
        let source = LocalDir::new(dir.path().to_path_buf());
        let fetched_manifest = source.latest_manifest().await.unwrap();

        let staging = dir.path().join("staging");
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        let fetched = source
            .fetch_artifact(&fetched_manifest.parsed, &staging, tx)
            .await
            .unwrap();

        assert_eq!(std::fs::read(&fetched.artifact).unwrap(), b"payload-bytes");
        assert_eq!(rx.recv().await, Some((13, Some(13))));
    }
}
