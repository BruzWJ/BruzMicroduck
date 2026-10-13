#!/bin/sh
# Install the `duckctl` a branch last built, on Linux or macOS, with no Rust toolchain.
#
#   curl -fsSL https://raw.githubusercontent.com/BruzWJ/BruzMicroduck/replica/scripts/install-duckctl.sh | sh
#   curl -fsSL .../install-duckctl.sh | sh -s -- udp-pad
#
# The binary comes from the `daemon-dev-<branch>` prerelease that `.github/workflows/dev.yml`
# replaces on every push; the branch defaults to `replica`. It goes to `$DUCKCTL_DIR`, by default
# `~/.local/bin`, which needs no sudo.
set -eu

REPO=BruzWJ/BruzMicroduck
branch="${1:-replica}"
dir="${DUCKCTL_DIR:-$HOME/.local/bin}"

case "$(uname -s)" in
    Linux) os=linux ;;
    Darwin) os=macos ;;
    *) echo "install-duckctl: $(uname -s) is not covered here; Windows takes the .zip" >&2; exit 1 ;;
esac
case "$(uname -m)" in
    x86_64 | amd64) arch=x86_64 ;;
    arm64 | aarch64)
        if [ "$os" = macos ]; then arch=arm64; else arch=aarch64; fi ;;
    *) echo "install-duckctl: no build for $(uname -m)" >&2; exit 1 ;;
esac

url="https://github.com/${REPO}/releases/download/daemon-dev-${branch}/duckctl-${os}-${arch}.tar.gz"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "duckctl from ${branch}: ${url}"
if ! curl -fsSL "$url" -o "$tmp/duckctl.tar.gz"; then
    echo "install-duckctl: no build of '${branch}' — has the branch been pushed since dev.yml" >&2
    echo "built duckctl, and has that run finished?" >&2
    exit 1
fi
tar -xzf "$tmp/duckctl.tar.gz" -C "$tmp"
mkdir -p "$dir"
mv "$tmp/duckctl" "$dir/duckctl"
chmod +x "$dir/duckctl"
echo "installed $dir/duckctl"

case ":${PATH}:" in
    *":${dir}:"*) ;;
    *) echo "$dir is not on your PATH — add it, or run $dir/duckctl" ;;
esac
