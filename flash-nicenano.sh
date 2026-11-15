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

set -euo pipefail

UF2="${1:-}"
HINT="${2-dabase--v2}"
DRIVE_LABEL="${DRIVE_LABEL:-NICENANO}"
BOOT_WAIT="${BOOT_WAIT:-40}"

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

shell_cmd() {
    # Send one command to the ZMK shell.
    #
    # Opening the tty drops the first character, and the shell's RX ring
    # overflows on bursty writes (it is shared with the log console), so the
    # command goes out byte by byte on a single open handle.
    local port="$1" cmd="$2" i
    configure_port "$port"
    exec 9<>"$port" || return 1
    # Writes can fail mid-command when the board resets into the bootloader;
    # that is the expected outcome, so never abort the flash on write errors.
    { printf '\r' >&9; } 2>/dev/null || true
    sleep 0.6
    for ((i = 0; i < ${#cmd}; i++)); do
        { printf '%s' "${cmd:i:1}" >&9; } 2>/dev/null || true
        sleep 0.03
    done
    { printf '\r' >&9; } 2>/dev/null || true
    sleep 0.3
    exec 9>&- 2>/dev/null || true
    return 0
}

wait_for_drive() {
    local deadline=$((SECONDS + $1))
    while ((SECONDS < deadline)); do
        if [[ -e "/dev/disk/by-label/$DRIVE_LABEL" ]]; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# ------------------------------------------------------------- bootloader --
say "=== nice_nano headless flasher ==="
say "[1/4] Entering bootloader (GPREGRET 0x57 + cold reboot)"

DRIVE=""
for attempt in 1 2; do
    if wait_for_drive 2; then
        say "      already in the bootloader"
        DRIVE="/dev/disk/by-label/$DRIVE_LABEL"
        break
    fi

    PORT=""
    if PORT=$(find_port); then
        say "      serial console: $PORT"
        shell_cmd "$PORT" "devmem 0x4000051C 32 0x00000057"
        shell_cmd "$PORT" "kernel reboot cold"
        say "      bootloader request sent (attempt $attempt)"
    else
        say "      WARN: no ZMK serial console found (hint '$HINT')"
    fi

    say "[2/4] Waiting for $DRIVE_LABEL volume"
    if wait_for_drive "$BOOT_WAIT"; then
        DRIVE="/dev/disk/by-label/$DRIVE_LABEL"
        break
    fi
    say "      not found yet, retrying the bootloader request"
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
