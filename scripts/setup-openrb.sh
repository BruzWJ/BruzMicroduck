#!/bin/sh
# Install the host-side half of the OpenRB-150 USB-to-Dynamixel transport.
#
# This runs from hooks/postinstall on every release (including the release bootstrapped by
# install.sh), and setup-board.sh uses the same helper on a fresh image. Keeping it in the release
# is what lets the hardware cutover reach boards that only ever update and never run
# setup-board.sh again.
#
# The OpenRB must run ROBOTIS' factory usb_to_dynamixel firmware. It enumerates as USB CDC with
# VID:PID 2f5d:2202; this rule gives that tty a stable path and keeps ModemManager from writing
# probes into the Dynamixel packet stream.
#
set -eu

OPENRB_RULE="${OPENRB_RULE:-/etc/udev/rules.d/99-robot-openrb.rules}"
ROBOTD_CONFIG="${ROBOTD_CONFIG:-/etc/robot/robotd.toml}"

say()  { printf '%s\n' "setup-openrb: $*"; }
warn() { printf '%s\n' "setup-openrb: warning: $*" >&2; }

install_rule() {
    content='SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{idProduct}=="2202", SYMLINK+="openrb-dxl", ENV{ID_MM_PORT_IGNORE}="1"'

    if [ -f "$OPENRB_RULE" ] && [ "$(cat "$OPENRB_RULE")" = "$content" ]; then
        say "/dev/openrb-dxl rule already in place"
    else
        mkdir -p "$(dirname "$OPENRB_RULE")"
        printf '%s\n' "$content" > "$OPENRB_RULE"
        chmod 644 "$OPENRB_RULE"
        say "installed the /dev/openrb-dxl rule"
    fi

    # A release hook also runs in build/test containers without udev. On a real board reload and
    # retrigger on every run, even when the file was already current: that recovers an attached
    # board from a prior transient reload failure or recreates a missing symlink. Leave the
    # persistent rule in place if udev is temporarily absent or busy; unplug/replug (or the next
    # boot) will apply it.
    if ! command -v udevadm >/dev/null 2>&1; then
        warn "udevadm is unavailable; reconnect the OpenRB-150 or reboot after udev is available"
        return 0
    fi
    if ! udevadm control --reload-rules; then
        warn "could not reload udev rules; reconnect the OpenRB-150 or reboot"
        return 0
    fi
    udevadm trigger --subsystem-match=tty 2>/dev/null \
        || warn "could not retrigger tty devices; reconnect the OpenRB-150"
    udevadm settle --timeout=10 2>/dev/null \
        || warn "udev did not settle; /dev/openrb-dxl may appear after reconnect"
}

migrate_robotd_config() {
    if [ ! -f "$ROBOTD_CONFIG" ]; then
        say "${ROBOTD_CONFIG} is absent; nothing to migrate"
        return 0
    fi

    # robotd.toml is operator-owned. Only the byte-exact default shipped by the old custom-HAT
    # transport is ours to migrate; whitespace changes and every other path are custom and stay
    # untouched.
    if grep -Fxq 'port = "/dev/ttyS2"' "$ROBOTD_CONFIG"; then
        say "migrating robotd motor port from /dev/ttyS2 to /dev/openrb-dxl"
        sed -i 's|^port = "/dev/ttyS2"$|port = "/dev/openrb-dxl"|' "$ROBOTD_CONFIG"
    else
        say "robotd motor port is already current or customized"
    fi
}

install_rule
migrate_robotd_config
