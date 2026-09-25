//! Where artifacts come from.
//!
//! One trait, three backends: GitHub Releases (daemon), HF Hub (models), and a
//! local directory. The local backend is not a toy — it is how the engine gets
//! tested against its real code path with no network
//! (`docs/design/updater-design.md` §16.1).

use std::path::{Path, PathBuf};

use crate::Error;
use crate::config::SourceConfig;
use crate::manifest::Manifest;

pub(crate) mod github;
mod hf_hub;
pub mod http;
mod local;

pub use github::GithubReleases;
pub use hf_hub::HfHub;
pub use local::LocalDir;

/// Reports download progress as `(bytes_so_far, total_if_known)`.
///
/// A plain channel sender rather than a closure: the download happens in a task,
/// and progress has to reach subscribers without borrowing the caller's stack.
pub type ProgressSink = tokio::sync::mpsc::UnboundedSender<(u64, Option<u64>)>;

#[async_trait::async_trait]
pub trait Source: Send + Sync {
    /// Fetch the manifest the source currently advertises as newest.
    ///
    /// Returns the raw bytes alongside the parsed manifest so the exact document
    /// supplied by the source can be retained with the installed release.
    async fn latest_manifest(&self) -> Result<FetchedManifest, Error>;

    /// Fetch the manifest for an exact version. Backs
    /// `robotctl update apply --version X`.
    async fn manifest_for(&self, version: &semver::Version) -> Result<FetchedManifest, Error>;

    /// Fetch the manifest a named ref currently points at. Backs
    /// `robotctl update apply --ref my-branch`.
    ///
    /// Defaulted rather than required because "a ref" is not meaningful for every source: a
    /// local directory has no refs, and a source that cannot resolve one should say so
    /// rather than guess. Sources that *can* override it.
    async fn manifest_at_ref(&self, git_ref: &str) -> Result<FetchedManifest, Error> {
        Err(Error::Incompatible(format!(
            "this source cannot resolve the ref {git_ref:?}; \
             only a github_releases source publishes per-branch builds"
        )))
    }

    /// Download the artifact into `dest_dir`.
    ///
    /// Streams to disk — must not assume the artifact fits in memory — and is
    /// cancel-safe: dropping the future must leave only staging garbage, never a
    /// partially installed release.
    async fn fetch_artifact(
        &self,
        manifest: &Manifest,
        dest_dir: &Path,
        progress: ProgressSink,
    ) -> Result<FetchedArtifact, Error>;
}

/// A manifest as received, plus its parsed representation.
///
/// Keeping both preserves the exact source document in the installed release
/// while letting the engine make typed compatibility decisions.
#[derive(Debug, Clone)]
pub struct FetchedManifest {
    pub bytes: Vec<u8>,
    pub parsed: Manifest,
}

#[derive(Debug, Clone)]
pub struct FetchedArtifact {
    pub artifact: PathBuf,
}

/// Build a source from config.
pub fn from_config(config: &SourceConfig) -> Box<dyn Source> {
    match config {
        SourceConfig::GithubReleases {
            repo,
            tag_prefix,
            manifest_asset,
            ref_tag_prefix,
        } => Box::new(GithubReleases::new(
            repo.clone(),
            tag_prefix.clone(),
            manifest_asset.clone(),
            ref_tag_prefix.clone(),
        )),
        SourceConfig::HfHub {
            repo,
            revision,
            manifest_file,
        } => Box::new(HfHub::new(
            repo.clone(),
            revision.clone(),
            manifest_file.clone(),
        )),
        SourceConfig::LocalDir { path } => Box::new(LocalDir::new(path.clone())),
    }
}
