# CI setup

Status: draft · Date: 2026-07-28 · Owner: pierre

One-time setup for the release pipeline. See [`updater-design.md`](../design/updater-design.md)
§5.4 for key custody and §16.3 for publishing.

## Decision: two signing keys and no gate on this plan

**Decided 2026-07-29.** Branch pushes are signed with `team.dev`; stable releases are signed with
`release-1`. Both keys live in CI.

| trigger | workflow | key | reaches a customer robot |
|---|---|---|---|
| push to any branch | `dev.yml` | `team.dev` (repo secret) | **no** — `allow_dev_keys = false` there, and the trusted filename must end `.dev.pub` |
| manual **Run workflow** | `release.yml` | `release-1` (`release` env secret) | **yes** — creates one stable release |

Dev builds stay dev builds: their key is not trusted by customer robots, and a stable release is
always rebuilt, signed, verified, tagged, and published by the manual release workflow.

### What was intended, and why it is not there

The plan was to gate `release-1` behind the `release` environment's required-reviewers rule.
It cannot be created:

```
HTTP 422: Failed to create the environment protection rule.
Please ensure the billing plan supports the required reviewers protection rule.
```

Tag protection was checked as a substitute and is also unavailable:

```
403: Upgrade to GitHub Pro or make this repository public to enable this feature.
```

Required reviewers, deployment branch policies, branch protection and rulesets are all
Team/Pro features on a *private* repository, and `pollen-robotics` is on the free plan. The
`release` environment exists with zero protection rules.

### The accepted risk, stated plainly

**Anyone with push access can read `release-1`.** Scoping it to the `release` environment
stops a workflow that does not declare that environment from seeing it, but any collaborator
can author one that does. "Used only for releases" is therefore a convention among people who
already trust each other, not an access control — the workflow file is not a boundary.

This was accepted deliberately: the team is small and mutually trusted, no robot has left the
building, and the alternative (signing every release by hand) buys nothing today against a
threat that does not yet exist.

**Revisit when either becomes true**, because the cost changes sharply and the failure is the
one this design cannot undo — a leaked key means shipping a `release-2`-signed update to every
robot, and any robot that misses it trusts the compromised key forever:

- a robot is in someone's home, or
- someone with push access is not someone you would hand the signing key to directly.

The fix at that point is upgrading the org to GitHub Team, which keeps this split and adds the
gate; or moving `release-1` signing back to a laptop.

## The tiering (unchanged, and still the thing that bounds damage)

Whatever is decided above, what limits the cost of a compromise is which key is reachable
from where:

| key | in CI | role |
|---|---|---|
| `release-1` | **not currently** — see above | signs every stable release |
| `release-2` | no | first rotation target if CI or `release-1` is compromised |
| `release-3` | no, ideally never on a networked machine | last resort |
| `team.dev` | intended, dev workflow only | branch builds; cannot touch a customer robot, because `allow_dev_keys` is false there |

All **public** keys go into every robot image from the start — a robot can only verify
against the set baked into it, so this is the only chance to make rotation possible
without physically re-flashing.

## Secrets and variables

GitHub Secrets are **write-only**: once set, nobody — including you — can read them back.
They are a *deployment copy*, never storage. The password manager remains the system of
record; losing it means the key is gone and every robot trusting it can never be signed
for again.

**Scope them to the `release` environment, not to the repository.** A repository secret is
readable by every workflow job in the repo; an environment secret is readable only by a job
declaring that environment. On this plan that difference stops an unrelated workflow from
seeing the key, and nothing more (see above) — but it is strictly better and costs nothing:

```bash
gh secret set MINISIGN_SECRET_KEY --env release < ~/.duck-keys/release-1.key
```

```bash
gh secret set MINISIGN_PASSWORD --env release
```

The second prompts, so the passphrase never lands in shell history or a transcript.

**Release secrets** (encrypted, not readable back):

| name | scope | value |
|---|---|---|
| `MINISIGN_SECRET_KEY` | `release` env | `~/.duck-keys/release-1.key`, both lines |
| `MINISIGN_PASSWORD` | `release` env | the passphrase for `release-1` |

The separate `MINISIGN_DEV_SECRET_KEY` is repo-scoped: every branch push signs with it, so
gating it behind an environment would mean the dev workflow declaring one meant for
`release-1`. It needs no passphrase secret — a dev key is unencrypted so CI can sign
non-interactively, which `xtask keycheck` confirms and calls correct for a dev key and wrong
for a release key.

The release verification job reads `deploy/trusted_keys/release-1.pub` directly, so there is no
public-key variable to configure and no duplicate value to drift. Do **not** add the private halves
of `release-2` or `release-3`; their value is being absent from CI.

## The `release` environment

The build job called by `release.yml` declares `environment: release`. Create it under Settings →
Environments and add **required reviewers**.

Without it, anyone who can run the release workflow can sign for the whole fleet.
With it, reaching the signing key needs a second person's approval — which recovers most
of what local signing would have given, at the cost of one click per release.

Fork pull requests never receive secrets, so the key is unreachable from contributor PRs
regardless.

## Where the key is handled

Exactly one step per workflow writes the key to disk, and it is removed immediately:

```
umask 077
printf '%s' "$MINISIGN_SECRET_KEY" > "$RUNNER_TEMP/secret.key"
cargo run -p xtask -- sign --dir dist --key "$RUNNER_TEMP/secret.key"
shred -u "$RUNNER_TEMP/secret.key" || rm -f "$RUNNER_TEMP/secret.key"
```

Written to a file rather than passed as an argument, because a key on a command line is
visible in the process list to anything else on the runner.

`release.yml`'s verification step deliberately needs **no** key: `xtask package` emits a
second manifest with a bare-filename URL (for `LocalDir`), and `xtask sign` signs both in
one pass. Re-signing to verify would mean handling the signing key twice in one job for
no benefit.

## Cutting a release

The normal release is a single GitHub action:

1. Bump `[workspace.package].version` in `Cargo.toml`, commit, and push the default branch.
2. Open **Actions → release → Run workflow**. Leave the branch selector on the default branch;
   there are no other inputs.
3. Click **Run workflow**.

`release.yml` reads the version, freezes the selected commit SHA, and calls the release recipe once.
The GitHub-hosted runner cross-builds for aarch64, packages, signs with `release-1`, verifies a real
install through `updaterd`, creates `daemon-v<version>` at that SHA, uploads every asset, and
publishes one stable/latest release. A normal release therefore needs no local command, manually
created tag, personal access token, `DUCK_TOKEN`, staging release, or promotion.

If that version is already published, the run refuses to replace it; bump the version. A retry may
resume only a draft for the same version and commit.

### The workflows

```
release.yml            manual entry point; derives the version and freezes the source SHA
_build-release.yml     build · package · sign · verify · publish        (called)
dev.yml                every push: an unsigned-for-customers dev build, `team.dev` key
```

`xtask`'s packaging tripwires read `_build-release.yml` and `dev.yml` for the `--include` list, so a
unit, hook or sysusers file that is not packaged fails a test rather than a robot.

The normal path creates the release with its assets, so `gh` uploads through an internal draft before
making it visible. If a run leaves a draft, the same commit can resume it; an already-published
manual release is never overwritten.

## Rotating a key

If `release-1` or CI is compromised:

1. Replace `MINISIGN_SECRET_KEY` / `MINISIGN_PASSWORD` with `release-2`'s.
2. Publish a release signed by `release-2`. Robots already trust it — that is why both
   public keys shipped from the first image.
3. Remove `release-1.pub` from `trusted_keys_dir` in a subsequent release, so the
   compromised key stops being accepted.
4. Generate a replacement third key so a spare still exists:
   `cargo xtask keygen --kind release --name release-4 --out ~/.duck-keys`

Step 3 lags step 2 on purpose: revoking the old key before every robot has taken the
new-signed release would strand any robot that missed it.
