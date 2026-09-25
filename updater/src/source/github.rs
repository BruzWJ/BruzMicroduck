//! GitHub Releases source — the daemon channel.
//!
//! "Latest" is resolved by listing releases and taking the highest **semver** among
//! tags matching `tag_prefix`, not by using `/releases/latest`: that endpoint is
//! repo-wide, so it breaks the moment a second channel shares the repo, and it
//! answers "most recently published" rather than "highest version" — which differ as
//! soon as you publish a patch to an older line.
//! See `docs/design/updater-design.md` §6.
//!
//! Release metadata and assets are accepted from GitHub over HTTPS. The engine
//! verifies the downloaded artifact against the SHA-256 recorded in the manifest
//! before extracting it.

use std::path::Path;

use serde::Deserialize;

use crate::Error;
use crate::manifest::Manifest;
use crate::source::{FetchedArtifact, FetchedManifest, ProgressSink, Source, http};

/// Releases fetched per page when scanning for the newest tag. One page is plenty
/// for any real channel; further pages are fetched only if a page comes back full.
const PER_PAGE: usize = 100;
const MAX_PAGES: usize = 5;

/// What the release-asset API needs to return bytes rather than JSON metadata.
const OCTET_STREAM: &str = "application/octet-stream";

pub struct GithubReleases {
    repo: String,
    tag_prefix: String,
    manifest_asset: String,
    ref_tag_prefix: String,
    client: reqwest::Client,
}

/// Only the fields we use. GitHub adds fields freely, so this is deliberately not
/// exhaustive.
#[derive(Debug, Deserialize)]
struct Release {
    /// What [`GithubReleases::release_for_tag`] re-asks by when the tag answer lists nothing.
    id: u64,
    tag_name: String,
    #[serde(default)]
    draft: bool,
    #[serde(default)]
    prerelease: bool,
    #[serde(default)]
    assets: Vec<Asset>,
}

#[derive(Debug, Deserialize)]
struct Asset {
    name: String,
    /// The API endpoint for this asset, `/repos/{owner}/{repo}/releases/assets/{id}`.
    ///
    /// Used in preference to `browser_download_url` because that one **404s on a private
    /// repository**, with or without a token — verified against this repo. The API endpoint
    /// serves the bytes with a token and `Accept: application/octet-stream`, and works for
    /// public repos too, so there is one path rather than two.
    url: String,
}

impl GithubReleases {
    pub fn new(
        repo: String,
        tag_prefix: String,
        manifest_asset: String,
        ref_tag_prefix: String,
    ) -> Self {
        Self {
            repo,
            tag_prefix,
            ref_tag_prefix,
            manifest_asset,
            // A failure here means a broken TLS setup, which is fatal for every
            // request anyway; fall back to a default client so construction stays
            // infallible and the error surfaces on first use.
            client: http::client().unwrap_or_default(),
        }
    }

    fn tag_for(&self, version: &semver::Version) -> String {
        format!("{}{}", self.tag_prefix, version)
    }

    /// The tag a named ref resolves to: `daemon-dev-` + the branch name.
    ///
    /// The ref is appended verbatim. Branch names are already valid git refs, so slashes in
    /// `feature/foo` need no handling — and rewriting them would resolve to a tag that does
    /// not exist, failing with "release not found" instead of anything informative.
    fn ref_tag_for(&self, git_ref: &str) -> String {
        format!("{}{}", self.ref_tag_prefix, git_ref)
    }

    /// The release behind `tag`, with its assets.
    ///
    /// **An empty asset list from the tag lookup is asked again by id.** From 2026-09-23 the
    /// unauthenticated `releases/tags/{tag}` answer — and the listing — carry no assets for any
    /// release published since, while `releases/{id}` for the same release lists every one. A
    /// board asks without a token, so it saw a finished build as an empty one and refused it,
    /// every time. The second request costs nothing on a release that really is empty, and it
    /// is what makes [`Error::ReleaseNotReady`] mean what it says.
    async fn release_for_tag(&self, tag: &str) -> Result<Release, Error> {
        let url = format!(
            "https://api.github.com/repos/{}/releases/tags/{tag}",
            self.repo
        );
        let release = self.release_at(&url, tag).await?;
        if !release.assets.is_empty() {
            return Ok(release);
        }

        let url = format!(
            "https://api.github.com/repos/{}/releases/{}",
            self.repo, release.id
        );
        tracing::debug!(%tag, id = release.id, "tag lookup listed no assets; asking by id");
        self.release_at(&url, tag).await
    }

    async fn release_at(&self, url: &str, tag: &str) -> Result<Release, Error> {
        let bytes = http::get_bytes(&self.client, url, Some("application/vnd.github+json")).await?;
        serde_json::from_slice(&bytes)
            .map_err(|e| Error::Network(format!("parsing release {tag}: {e}")))
    }

    /// Highest **stable** semver among matching tags.
    ///
    /// Drafts and prereleases are skipped: a draft isn't published, and a prerelease is
    /// by definition not what a client robot should install. Reach one with an explicit
    /// `--version` or `--ref`.
    ///
    /// Two independent reasons a build is skipped — GitHub's `prerelease` flag *and* a
    /// semver prerelease component — because dev builds (`0.2.0-dev.5.abc1234`) must
    /// never become `latest` for the fleet, and relying on someone remembering a
    /// checkbox is not a safeguard.
    async fn newest_version(&self) -> Result<semver::Version, Error> {
        let mut best: Option<semver::Version> = None;

        for page in 1..=MAX_PAGES {
            let url = format!(
                "https://api.github.com/repos/{}/releases?per_page={PER_PAGE}&page={page}",
                self.repo
            );
            let bytes =
                http::get_bytes(&self.client, &url, Some("application/vnd.github+json")).await?;
            let releases: Vec<Release> = serde_json::from_slice(&bytes)
                .map_err(|e| Error::Network(format!("parsing releases: {e}")))?;

            let count = releases.len();
            for release in releases {
                if release.draft || release.prerelease {
                    continue;
                }
                if let Some(version) = version_under(&self.tag_prefix, &release.tag_name)
                    // A semver prerelease is a dev build, whatever the release was flagged as.
                    && version.pre.is_empty()
                    && best.as_ref().is_none_or(|b| version > *b)
                {
                    best = Some(version);
                }
            }

            // A short page is the last page.
            if count < PER_PAGE {
                break;
            }
        }

        best.ok_or_else(|| {
            Error::Network(format!(
                "no releases in {} with tag prefix {:?}",
                self.repo, self.tag_prefix
            ))
        })
    }

    /// The API download URL for a named asset. Pair it with [`OCTET_STREAM`].
    ///
    /// A release with *no* assets at all is reported separately, because it is not the same
    /// situation as a missing one. Assets are uploaded at the end of a release build, so an
    /// empty release is every release for the few minutes before that finishes — a state to
    /// wait out, not a fault to investigate. A release that has assets but not this one is the
    /// fault case, and there the list of what *is* there is the whole diagnostic.
    fn asset_url(&self, release: &Release, name: &str) -> Result<String, Error> {
        if release.assets.is_empty() {
            return Err(Error::ReleaseNotReady {
                repo: self.repo.clone(),
                tag: release.tag_name.clone(),
            });
        }

        release
            .assets
            .iter()
            .find(|a| a.name == name)
            .map(|a| a.url.clone())
            .ok_or_else(|| {
                let available: Vec<_> = release.assets.iter().map(|a| a.name.as_str()).collect();
                Error::Network(format!(
                    "release {} has no asset named {name:?} (has: {})",
                    release.tag_name,
                    available.join(", ")
                ))
            })
    }

    /// Split one of our own release-download URLs into `(tag, asset name)`.
    ///
    /// `None` for anything else, including another repository's release URL — which matters
    /// because the manifest this comes from is unverified at that point, so a URL naming a
    /// foreign repo must not become an authenticated API request against it.
    fn split_release_url(&self, url: &str) -> Option<(String, String)> {
        let prefix = format!("https://github.com/{}/releases/download/", self.repo);
        let (tag, name) = url.strip_prefix(&prefix)?.split_once('/')?;
        Some((tag.to_owned(), name.to_owned()))
    }

    /// Where to actually fetch a URL from a release manifest, and with which `Accept`.
    ///
    /// A private repo's `releases/download/...` URL 404s even with a token, so one of ours is
    /// re-resolved through the release API. Anything else is fetched verbatim — a manifest
    /// pointing at an HTTPS CDN keeps working, and the bytes are hash-checked either way.
    async fn resolve_download(&self, url: &str) -> Result<(String, Option<&'static str>), Error> {
        let Some((tag, name)) = self.split_release_url(url) else {
            if !url.starts_with("https://") {
                return Err(Error::Verification(format!(
                    "release artifact URL must use HTTPS, got {url:?}"
                )));
            }
            return Ok((url.to_owned(), None));
        };

        let release = self.release_for_tag(&tag).await?;
        let api_url = self.asset_url(&release, &name)?;
        tracing::debug!(%tag, %name, "resolved asset through the release API");
        Ok((api_url, Some(OCTET_STREAM)))
    }

    async fn manifest(&self, tag: &str) -> Result<FetchedManifest, Error> {
        let release = self.release_for_tag(tag).await?;

        let manifest_url = self.asset_url(&release, &self.manifest_asset)?;
        let bytes = http::get_bytes(&self.client, &manifest_url, Some(OCTET_STREAM)).await?;

        let parsed: Manifest = serde_json::from_slice(&bytes)
            .map_err(|e| Error::Corrupt(format!("manifest at {manifest_url}: {e}")))?;

        Ok(FetchedManifest { bytes, parsed })
    }
}

#[async_trait::async_trait]
impl Source for GithubReleases {
    async fn latest_manifest(&self) -> Result<FetchedManifest, Error> {
        let version = self.newest_version().await?;
        let tag = self.tag_for(&version);
        tracing::debug!(repo = %self.repo, %tag, "resolved latest");
        self.manifest(&tag).await
    }

    async fn manifest_for(&self, version: &semver::Version) -> Result<FetchedManifest, Error> {
        self.manifest(&self.tag_for(version)).await
    }

    async fn manifest_at_ref(&self, git_ref: &str) -> Result<FetchedManifest, Error> {
        let tag = self.ref_tag_for(git_ref);
        tracing::debug!(repo = %self.repo, %tag, %git_ref, "resolving ref");
        self.manifest(&tag).await
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

        // Treat the filename as untrusted and refuse anything that is not a bare name.
        let artifact_name = safe_file_name(&manifest.url)?;
        let artifact = dest_dir.join(&artifact_name);

        let (artifact_url, accept) = self.resolve_download(&manifest.url).await?;
        http::download_to(&self.client, &artifact_url, &artifact, accept, &progress).await?;

        Ok(FetchedArtifact { artifact })
    }
}

/// Parse a version out of a tag carrying `prefix`, or `None` if the tag is not one of ours.
///
/// Free-standing so tests can pin that only tags with the configured prefix are releases.
fn version_under(prefix: &str, tag: &str) -> Option<semver::Version> {
    semver::Version::parse(tag.strip_prefix(prefix)?).ok()
}

/// Extract a filename from a URL, refusing anything that could escape `dest_dir`.
///
/// A manifest is remote input, so it must never choose an arbitrary local path.
pub(crate) fn safe_file_name(url: &str) -> Result<String, Error> {
    let tail = url.rsplit('/').next().unwrap_or_default();
    let tail = tail.split(['?', '#']).next().unwrap_or_default();

    let looks_safe = !tail.is_empty()
        && tail != "."
        && tail != ".."
        && !tail.contains('/')
        && !tail.contains('\\')
        && !tail.contains('\0');

    if looks_safe {
        Ok(tail.to_owned())
    } else {
        Err(Error::Verification(format!(
            "manifest url {url:?} does not end in a usable filename"
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source() -> GithubReleases {
        GithubReleases::new(
            "ORG/robot-daemon".into(),
            "daemon-v".into(),
            "manifest.json".into(),
            "daemon-dev-".into(),
        )
    }

    #[test]
    fn tags_round_trip() {
        let s = source();
        let v = semver::Version::new(1, 4, 2);
        assert_eq!(s.tag_for(&v), "daemon-v1.4.2");
        assert_eq!(version_under(&s.tag_prefix, "daemon-v1.4.2"), Some(v));
    }

    /// A ref becomes a dev tag, and the ref is appended verbatim.
    ///
    /// The slash case is the one that matters: `feature/foo` is a valid branch name, so
    /// anything that sanitised it would resolve to a tag nobody published and fail with
    /// "release not found" rather than anything that points at the cause.
    #[test]
    fn refs_become_dev_tags_verbatim() {
        let s = source();
        assert_eq!(s.ref_tag_for("my-branch"), "daemon-dev-my-branch");
        assert_eq!(s.ref_tag_for("feature/foo"), "daemon-dev-feature/foo");
    }

    /// **A dev tag must never be mistaken for a release.** `version_under` drives
    /// `newest_version`, which is what the fleet installs — so if a dev tag parsed as a
    /// version here, a branch build could become `latest` for every robot. That is the
    /// failure the two independent guards exist to prevent, and this is the first of them.
    #[test]
    fn a_dev_tag_is_not_a_release_version() {
        let s = source();
        assert_eq!(version_under(&s.tag_prefix, "daemon-dev-my-branch"), None);
        // Even when the dev tag ends in something version-shaped.
        assert_eq!(version_under(&s.tag_prefix, "daemon-dev-0.2.0"), None);
    }

    /// Another channel's tags in the same repo must be ignored, not misparsed.
    #[test]
    fn foreign_tags_are_ignored() {
        let s = source();
        assert_eq!(version_under(&s.tag_prefix, "model-v3.0.0"), None);
        assert_eq!(version_under(&s.tag_prefix, "v1.0.0"), None);
        assert_eq!(version_under(&s.tag_prefix, "daemon-vnot-a-version"), None);
    }

    #[test]
    fn asset_lookup_lists_what_was_available_on_failure() {
        let release = Release {
            id: 1,
            tag_name: "daemon-v1.0.0".into(),
            draft: false,
            prerelease: false,
            assets: vec![Asset {
                name: "other.txt".into(),
                url: "https://api.github.com/repos/ORG/robot-daemon/releases/assets/1".into(),
            }],
        };
        let err = source().asset_url(&release, "manifest.json").unwrap_err();
        // A support ticket needs to see what *was* there.
        assert!(err.to_string().contains("other.txt"), "{err}");
    }

    /// A release whose build has not uploaded yet must not read as a broken release.
    ///
    /// This is what an operator sees if a release becomes visible before its assets finish
    /// uploading. The old answer — `network error: ... has no asset named "manifest.json"
    /// (has: )` — pointed at the two things that were not wrong: the release, and the network.
    #[test]
    fn an_empty_release_says_its_build_has_not_finished() {
        let release = Release {
            id: 1,
            tag_name: "daemon-v0.5.1".into(),
            draft: false,
            prerelease: true,
            assets: vec![],
        };

        let err = source().asset_url(&release, "manifest.json").unwrap_err();
        assert!(
            matches!(err, Error::ReleaseNotReady { .. }),
            "an empty release is a wait, not a fetch failure, got {err:?}"
        );

        let msg = err.to_string();
        // The tag, so it is clear *which* release, and where to watch it land.
        assert!(msg.contains("daemon-v0.5.1"), "{msg}");
        assert!(
            msg.contains("https://github.com/ORG/robot-daemon/releases/tag/daemon-v0.5.1"),
            "{msg}"
        );
        // And none of the vocabulary that sent people debugging the wrong thing.
        assert!(!msg.contains("network"), "{msg}");
        assert!(!msg.contains("no asset named"), "{msg}");
    }

    /// **A manifest must not redirect the asset lookup at another repository.**
    ///
    /// A URL naming a foreign repository must be left alone rather than turned into an API
    /// request carrying our token.
    #[test]
    fn only_our_own_release_urls_are_split() {
        let s = source();

        assert_eq!(
            s.split_release_url(
                "https://github.com/ORG/robot-daemon/releases/download/daemon-dev-my-branch/daemon-0.2.0-dev.1.abc1234.tar.zst"
            ),
            Some((
                "daemon-dev-my-branch".to_owned(),
                "daemon-0.2.0-dev.1.abc1234.tar.zst".to_owned()
            ))
        );

        for foreign in [
            "https://github.com/attacker/repo/releases/download/v1/x.tar.zst",
            "https://cdn.example.com/daemon-1.0.0.tar.zst",
            // Ours, but not a release-download URL.
            "https://github.com/ORG/robot-daemon/archive/refs/heads/main.tar.gz",
        ] {
            assert_eq!(s.split_release_url(foreign), None, "{foreign}");
        }
    }

    #[test]
    fn accepts_a_plain_filename() {
        assert_eq!(
            safe_file_name("https://example.com/a/b/daemon-1.0.0.tar.zst").unwrap(),
            "daemon-1.0.0.tar.zst"
        );
        // Query strings are common on signed CDN URLs.
        assert_eq!(
            safe_file_name("https://example.com/x.tar.zst?token=abc").unwrap(),
            "x.tar.zst"
        );
    }

    /// The download path must not be steerable by a manifest.
    #[test]
    fn refuses_names_that_could_escape() {
        for url in [
            "https://example.com/",
            "https://example.com/..",
            "https://example.com/.",
            "https://example.com/a/",
        ] {
            assert!(safe_file_name(url).is_err(), "should refuse {url}");
        }
    }

    /// A dev build must never be selected as `latest`, even if whoever published it
    /// forgot to tick "prerelease". Fleet-wide auto-updates read `latest`.
    #[test]
    fn dev_versions_are_recognised_as_prereleases() {
        let s = source();
        let dev = version_under(&s.tag_prefix, "daemon-v0.2.0-dev.5.abc1234").unwrap();
        assert!(
            !dev.pre.is_empty(),
            "dev builds must carry a semver prerelease"
        );

        let stable = version_under(&s.tag_prefix, "daemon-v0.2.0").unwrap();
        assert!(stable.pre.is_empty());
        // And a dev build sorts *below* the release it precedes, so it can never look
        // like an upgrade from it.
        assert!(dev < stable);
    }

    /// GitHub adds response fields regularly; deserialisation must not break.
    #[test]
    fn unknown_release_fields_are_tolerated() {
        let json = serde_json::json!({
            "id": 7,
            "tag_name": "daemon-v1.0.0",
            "some_new_field": 42,
            "assets": [{
                "name": "manifest.json",
                "url": "https://api.github.com/repos/ORG/robot-daemon/releases/assets/7",
                "browser_download_url": "https://example/m.json",
                "another_new_field": true
            }]
        });
        let release: Release = serde_json::from_value(json).unwrap();
        assert_eq!(release.tag_name, "daemon-v1.0.0");
        assert!(!release.draft, "missing `draft` should default to false");
        // `url` is required, not defaulted: it is how assets are fetched, and a release whose
        // assets lack it is not something to paper over with an empty string that would fail
        // later as a confusing HTTP error.
        assert!(release.assets[0].url.contains("/releases/assets/"));
    }
}
