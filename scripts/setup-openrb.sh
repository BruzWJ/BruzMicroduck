#!/bin/sh
# Install the host-side half of the OpenRB-150 USB-to-Dynamixel transport.
#
# `setup-board.sh` uses this helper while provisioning a freshly flashed image.
#
# The OpenRB must run ROBOTIS' factory usb_to_dynamixel firmware. It enumerates as USB CDC with
# VID:PID 2f5d:2202; this rule gives that tty a stable path and keeps ModemManager from writing
# probes into the Dynamixel packet stream.
#
set -eu

OPENRB_RULE="${OPENRB_RULE:-/etc/udev/rules.d/99-robot-openrb.rules}"

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

    # Tests and non-systemd images may have no udevadm. On a real board reload and retrigger even
    # when the file was already current: that recovers an attached controller from a transient
    # reload failure or recreates a missing symlink. Leave the persistent rule in place if udev is
    # temporarily absent or busy; unplug/replug (or the next boot) will apply it.
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

install_rule
