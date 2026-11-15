#!/usr/bin/env bash
# Linux port of flash-nicenano.ps1.
#
# Sends the nice_nano into its UF2 bootloader over the ZMK USB shell, waits
# for the NICENANO mass-storage volume, copies the UF2, and waits for the
# firmware's serial console to come back.
#
# Usage:
#   ./flash-nicenano.sh /path/to/firmware.uf2 [usb-id-hint]
#
# usb-id-hint (default: dabase--v2) selects the board among several
# /dev/serial/by-id/usb-ZMK_Project_* entries. Pass "" to use the only one.
#
# Env:
#   DRIVE_LABEL   mass-storage label (default NICENANO)
#   BOOT_WAIT     seconds to wait for the bootloader drive (default 40)
#   SHELL_CMD_WAIT  seconds to wait for the shell to answer (default 10)

set -euo pipefail

UF2="${1:-}"
HINT="${2-dabase--v2}"
DRIVE_LABEL="${DRIVE_LABEL:-NICENANO}"
BOOT_WAIT="${BOOT_WAIT:-40}"
SHELL_CMD_WAIT="${SHELL_CMD_WAIT:-10}"

if [[ -z "$UF2" || ! -f "$UF2" ]]; then
    echo "ERROR: UF2 not found: ${UF2:-<none>}" >&2
    echo "usage: $0 /path/to/firmware.uf2 [usb-id-hint]" >&2
    exit 1
fi

SIZE=$(stat -c %s "$UF2")
NAME=$(basename "$UF2")

say() { echo "$*"; }

# ---------------------------------------------------------------- serial --
find_port() {
    local byid
    for byid in /dev/serial/by-id/*ZMK_Project* /dev/serial/by-id/*zmk*; do
        [[ -e "$byid" ]] || continue
        byid=$(basename "$byid")
        if [[ -z "$HINT" ]] || [[ "$byid" == *"$HINT"* ]]; then
            readlink -f "/dev/serial/by-id/$byid"
            return 0
        fi
    done
    return 1
}

configure_port() {
    # USB CDC ignores the baud rate, but set it anyway; raw mode so the
    # shell sees our line endings untouched.
    stty -F "$1" 115200 raw -echo -echoe -echok 2>/dev/null || true
}

drain() {
    # Read whatever the console has buffered, for a short while.
    timeout 1 cat "$1" >/dev/null 2>&1 || true
}

shell_cmd() {
    # Wait until the shell answers, then run one command.
    local port="$1" cmd="$2" i
    for ((i = 0; i < SHELL_CMD_WAIT; i++)); do
        configure_port "$port"
        printf '\r\n' > "$port" 2>/dev/null || { sleep 1; continue; }
        sleep 0.4
        drain "$port"
        if printf '%s\r\n' "$cmd" > "$port"; then
            sleep 0.2
            return 0
        fi
        sleep 1
    done
    return 1
}

# ------------------------------------------------------------- bootloader --
say "=== nice_nano headless flasher ==="
say "[1/4] Entering bootloader (GPREGRET 0x57 + cold reboot)"

PORT=""
if PORT=$(find_port); then
    say "      serial console: $PORT"
    if shell_cmd "$PORT" "devmem 0x4000051C 32 0x00000057"; then
        shell_cmd "$PORT" "kernel reboot cold" || true
        say "      bootloader request sent"
    else
        say "      WARN: serial console did not answer - continuing anyway"
    fi
else
    say "      WARN: no ZMK serial console found (hint '$HINT') - continuing anyway"
fi

say "[2/4] Waiting for $DRIVE_LABEL volume"
DRIVE=""
DEADLINE=$((SECONDS + BOOT_WAIT))
while ((SECONDS < DEADLINE)); do
    if [[ -e "/dev/disk/by-label/$DRIVE_LABEL" ]]; then
        DRIVE="/dev/disk/by-label/$DRIVE_LABEL"
        break
    fi
    sleep 1
done

if [[ -z "$DRIVE" ]]; then
    echo "ERROR: $DRIVE_LABEL volume not found after ${BOOT_WAIT}s" >&2
    exit 1
fi
say "      found $DRIVE -> $(readlink -f "$DRIVE")"

# ---------------------------------------------------------------- copy --
say "[3/4] Copying $NAME ($SIZE bytes)"

mountpoint_of() {
    lsblk -no MOUNTPOINT "$1" 2>/dev/null | head -1
}

MNT="$(mountpoint_of "$DRIVE")"
if [[ -z "$MNT" ]]; then
    udisksctl mount -b "$DRIVE" >/dev/null 2>&1 || true
    sleep 1
    MNT="$(mountpoint_of "$DRIVE")"
fi
if [[ -z "$MNT" ]]; then
    echo "ERROR: could not mount $DRIVE" >&2
    exit 1
fi
say "      mounted at $MNT"

for attempt in 1 2 3; do
    # The bootloader drops the volume as soon as it accepts the image, so a
    # copy error is expected and is not a failure.
    cp "$UF2" "$MNT/" 2>/dev/null || true
    sync
    sleep 2
    if [[ ! -e "/dev/disk/by-label/$DRIVE_LABEL" ]]; then
        say "      volume disconnected - bootloader accepted the image, flashing"
        break
    fi
    if [[ -e "$MNT/$NAME" ]] && [[ "$(stat -c %s "$MNT/$NAME")" == "$SIZE" ]]; then
        say "      copy verified on the volume"
        break
    fi
    say "      copy attempt $attempt incomplete - retrying"
    if [[ "$attempt" == 3 ]]; then
        echo "ERROR: copy failed after 3 attempts" >&2
        exit 1
    fi
    sleep 2
done

# -------------------------------------------------------------- reboot --
say "[4/4] Waiting for the firmware console to come back"
DEADLINE=$((SECONDS + 30))
while ((SECONDS < DEADLINE)); do
    if PORT=$(find_port); then
        say "      device back online on $PORT"
        exit 0
    fi
    sleep 1
done
say "      WARN: console did not come back within 30s (drive disconnected - flash may still have succeeded)"
exit 0
