//! Hugging Face Hub source — the model channel.
//!
//! Files resolve at `https://huggingface.co/{repo}/resolve/{revision}/{file}`.
//! Artifacts are accepted over HTTPS and checked against the SHA-256 in their
//! manifest before extraction.
//!
//! Two things differ from GitHub Releases and shape the code:
//!
//!  - There is no "releases" concept, only git revisions. `revision` in config is
//!    typically a moving branch, so "latest" means "whatever that branch points at
//!    right now" — and the *manifest's own* `version` field is authoritative for
//!    what we actually installed, never the branch name.
//!  - Exact versions map to tags by convention (`v1.2.3`), so an exact fetch is the
//!    same code path with a different revision.

use std::path::Path;

use serde::Deserialize;

use crate::Error;
use crate::manifest::Manifest;
use crate::source::{FetchedArtifact, FetchedManifest, ProgressSink, Source, http};

pub struct HfHub {
    repo: String,
    revision: String,
    manifest_file: String,
    client: reqwest::Client,
}

/// Subset of the HF repo API used to enumerate tags for an exact-version lookup.
#[derive(Debug, Deserialize)]
struct RepoRefs {
    #[serde(default)]
    tags: Vec<GitRef>,
}

#[derive(Debug, Deserialize)]
struct GitRef {
    name: String,
}

impl HfHub {
    pub fn new(repo: String, revision: String, manifest_file: String) -> Self {
        Self {
            repo,
            revision,
            manifest_file,
            client: http::client().unwrap_or_default(),
        }
    }

    fn resolve_url(&self, revision: &str, file: &str) -> String {
        format!(
            "https://huggingface.co/{}/resolve/{revision}/{file}",
            self.repo
        )
    }

    /// Tag naming convention for an exact version.
    fn tag_for(version: &semver::Version) -> String {
        format!("v{version}")
    }

    async fn manifest(&self, revision: &str) -> Result<FetchedManifest, Error> {
        let manifest_url = self.resolve_url(revision, &self.manifest_file);

        let bytes = http::get_bytes(&self.client, &manifest_url, None).await?;

        let parsed: Manifest = serde_json::from_slice(&bytes)
            .map_err(|e| Error::Corrupt(format!("manifest at {manifest_url}: {e}")))?;

        Ok(FetchedManifest { bytes, parsed })
    }

    /// Does the repo have a tag for this version?
    ///
    /// Checked so a missing version fails with "no tag v1.2.3 (available: …)" rather
    /// than a bare 404 from the resolve endpoint.
    async fn tag_exists(&self, tag: &str) -> Result<bool, Error> {
        let url = format!("https://huggingface.co/api/models/{}/refs", self.repo);
        let bytes = match http::get_bytes(&self.client, &url, None).await {
            Ok(bytes) => bytes,
            // The refs API is a convenience; if it's unavailable, fall through and let
            // the resolve attempt produce the error.
            Err(e) => {
                tracing::debug!(error = %e, "could not list refs; skipping the tag check");
                return Ok(true);
            }
        };
        let refs: RepoRefs = serde_json::from_slice(&bytes)
            .map_err(|e| Error::Network(format!("parsing refs for {}: {e}", self.repo)))?;
        Ok(refs.tags.iter().any(|t| t.name == tag))
    }
}

#[async_trait::async_trait]
impl Source for HfHub {
    async fn latest_manifest(&self) -> Result<FetchedManifest, Error> {
        // `revision` is usually a moving branch, so this is "whatever it points at
        // now". The manifest's `version` is what we record as installed.
        self.manifest(&self.revision).await
    }

    async fn manifest_for(&self, version: &semver::Version) -> Result<FetchedManifest, Error> {
        let tag = Self::tag_for(version);
        if !self.tag_exists(&tag).await? {
            return Err(Error::Network(format!(
                "{} has no tag {tag} for version {version}",
                self.repo
            )));
        }
        let fetched = self.manifest(&tag).await?;

        // A tag whose manifest disagrees with it means the publisher tagged the wrong
        // commit. Refusing beats installing something other than what was asked for.
        if fetched.parsed.version != *version {
            return Err(Error::Corrupt(format!(
                "tag {tag} contains a manifest for version {} — the tag and manifest disagree",
                fetched.parsed.version
            )));
        }
        Ok(fetched)
    }

    async fn fetch_artifact(
        &self,
        manifest: &Manifest,
        dest_dir: &Path,
        progress: ProgressSink,
    ) -> Result<FetchedArtifact, Error> {
        tokio::fs::create_dir_all(dest_dir)
            .await
            .map_err(|e| Error::Io {
                path: dest_dir.to_path_buf(),
                source: e,
            })?;

        // A model manifest's `url` is a path *within the repo*, unlike GitHub's
        // absolute asset URLs — so resolve it against the same revision the manifest
        // came from, keeping the artifact and its manifest consistent.
        let artifact_url = if manifest.url.starts_with("https://") {
            manifest.url.clone()
        } else if manifest.url.contains("://") {
            return Err(Error::Verification(format!(
                "model artifact URL must use HTTPS, got {:?}",
                manifest.url
            )));
        } else {
            self.resolve_url(&self.revision, &manifest.url)
        };

        // Remote input must not choose an arbitrary local path.
        let artifact_name = super::github::safe_file_name(&manifest.url)?;
        let artifact = dest_dir.join(&artifact_name);

        http::download_to(&self.client, &artifact_url, &artifact, None, &progress).await?;

        Ok(FetchedArtifact { artifact })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source() -> HfHub {
        HfHub::new(
            "ORG/gait-model".into(),
            "main".into(),
            "manifest.json".into(),
        )
    }

    #[test]
    fn resolve_url_matches_the_hub_layout() {
        assert_eq!(
            source().resolve_url("main", "manifest.json"),
            "https://huggingface.co/ORG/gait-model/resolve/main/manifest.json"
        );
    }

    #[test]
    fn exact_versions_map_to_v_prefixed_tags() {
        assert_eq!(HfHub::tag_for(&semver::Version::new(3, 1, 0)), "v3.1.0");
    }

    #[test]
    fn refs_parsing_tolerates_extra_fields() {
        let json = serde_json::json!({
            "branches": [{ "name": "main", "targetCommit": "abc" }],
            "tags": [{ "name": "v1.0.0", "targetCommit": "def" }],
            "converts": []
        });
        let refs: RepoRefs = serde_json::from_value(json).unwrap();
        assert_eq!(refs.tags.len(), 1);
        assert_eq!(refs.tags[0].name, "v1.0.0");
    }

    #[test]
    fn missing_tags_field_is_not_an_error() {
        let refs: RepoRefs = serde_json::from_value(serde_json::json!({})).unwrap();
        assert!(refs.tags.is_empty());
    }
}
