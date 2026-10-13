# CI setup

Status: current · Date: 2026-09-24

The stable-release pipeline needs no repository secrets, signing keys, protected environment or
local release tooling. The release and update mechanism is owned by
[`updater-design.md`](../design/updater-design.md); this page records the GitHub repository setup
and the operator's procedure.

## Repository setup

Keep GitHub Actions enabled and allow the repository `GITHUB_TOKEN` to receive the permissions
declared by the workflow. `.github/workflows/release.yml` grants its release job
`contents: write`; every other permission stays at the workflow's `contents: read` default.

There are no release secrets or variables to add. In particular, do not create signing keys,
password secrets, a release environment or a personal access token for this workflow.

GitHub's repository role is the authorization boundary: only a user with write access can run a
`workflow_dispatch` workflow. The workflow also refuses a branch selector other than the
repository's default branch. Use repository access settings to decide who may publish; no separate
in-workflow approver list exists.

## Cutting a release

1. Bump `[workspace.package].version` in `Cargo.toml` to a stable `X.Y.Z` version, commit, and push
   the default branch.
2. Open **Actions → release → Run workflow**, leave the branch selector on the default branch,
   and click **Run workflow**. There are no inputs.
3. Wait for the single job to finish. Do not create the tag or GitHub release first.

The workflow's draft, asset, digest, tag, retry and immutability rules are specified once in
[`updater-design.md` §16.3](../design/updater-design.md#163-release-publishing). `DUCK_TOKEN`, a
personal access token and a locally created tag are not part of this procedure.

## Existing development boards

A development board running the former unpublished updater must start from a clean OS image before
it is provisioned against the first release in the current format. The boundary and the reason a
plain reprovision is not a migration are owned by
[`updater-design.md` §5.4](../design/updater-design.md#54-publication-authority-and-integrity).

## Workflow inventory

- `release.yml` is the sole manual recipe for a complete stable daemon release.
- `dev.yml` publishes the hash-verified moving development release for every branch push.
- `_build-duckctl.yml` builds the cross-platform `duckctl` archives and wheels used by development
  releases and PyPI.
- `duckctl-pypi.yml` publishes stable `duckctl` wheels to PyPI from a `daemon-vX.Y.Z` tag.

The release-shape tripwires in `xtask` inspect both `release.yml` and `dev.yml`; keep their binary
lists aligned with `deploy/updater.toml` when a daemon is added or removed.

## duckctl on PyPI

PyPI publishing uses trusted publishing, not a repository secret. Before the first upload, an owner
of the chosen PyPI distribution must configure a GitHub publisher for this repository, the
`duckctl-pypi.yml` workflow and the `pypi` environment. The package currently declares the PyPI
distribution name `duckctl`, so that name must be available to or administered by this project's
maintainers before the workflow can publish it.

The manual release workflow creates its tag with `GITHUB_TOKEN`. GitHub deliberately does not start
another workflow from that event, so after `release.yml` succeeds, open **Actions → duckctl to
PyPI → Run workflow** and select the new `daemon-vX.Y.Z` tag. A retry is safe: the publisher skips
files that PyPI already accepted. A stable tag pushed outside Actions also triggers the workflow
directly.
