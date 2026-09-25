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

The one-time cutover for a development board running the former updater is
[`updater-design.md` §5.4](../design/updater-design.md#54-publication-authority-and-integrity):
reprovision or force-bootstrap it before applying the first release in the current format.
