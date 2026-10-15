#!/usr/bin/env bash
# link_serial_devices.sh — recreate the ttyMOTOR/ttyLIDAR symlinks INSIDE the
# Isaac ROS container.
#
# WHY THIS IS NEEDED
#   The udev rules on the host (tools/99-rover-serial.rules) create stable
#   names, but the container is started with --device for individual nodes, so
#   it sees /dev/ttyACM0 and NOT the symlinks.  udev does not run in the
#   container, so the rules cannot simply be installed there.
#
#   This script does what udev would: read each device's serial number out of
#   sysfs and link the right name to it.  Same matching, no udev.
#
# WHY IT MATTERS
#   ttyACM numbers are assigned in enumeration order.  With a second Espressif
#   board on the bus the motor/IMU bridge can change number between boots, and
#   rover_bringup.launch.py would open the wrong device with nothing in any log
#   naming the cause.
#
# RUN IT AFTER EVERY CONTAINER START, BEFORE STARTING NAV:
#     sudo bash src/rover_bringup_extended_slam/tools/link_serial_devices.sh
#
# THE PERMANENT FIX is to pass the symlink into the container instead, next to
# the --device /dev/i2c-7 entry:
#     --device /dev/ttyMOTOR:/dev/ttyMOTOR
# Docker resolves the symlink at start and creates a node at that path inside.
# Then this script becomes unnecessary.

set -u

# Your motor/IMU ESP32-S3's USB serial (it reports its MAC address).  Find it
# on the host with:  udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT
MOTOR_SERIAL="PUT-YOUR-MOTOR-ESP32-SERIAL-HERE"

link() {  # link <target-dev> <name>
    ln -sfn "$1" "/dev/$2" && echo "  /dev/$2 -> $1"
}

echo "scanning USB serial devices..."

for dev in /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    name=$(basename "$dev")
    # The serial number lives on the USB device, a couple of levels up from the
    # tty in sysfs.  Walk up until a 'serial' attribute appears.
    syspath="/sys/class/tty/$name/device"
    serial=""
    for _ in 1 2 3 4 5; do
        if [ -r "$syspath/serial" ]; then
            serial=$(cat "$syspath/serial")
            break
        fi
        syspath="$syspath/.."
    done

    case "$serial" in
        "$MOTOR_SERIAL") link "$dev" ttyMOTOR ;;
        "") echo "  $dev: no serial attribute found (skipped)" ;;
        *)  echo "  $dev: serial $serial — not a known board (skipped)" ;;
    esac
done

# The LiDAR's CP2102 has a unit-specific serial, so match on the chip instead:
# there is only ever one of them on this rover.
#
# ttyUSB nests one level deeper in sysfs than ttyACM — the tty hangs off a
# usb-serial PORT, which hangs off the interface, which hangs off the USB
# device that actually carries idVendor.  So walk up rather than assuming a
# fixed depth; the walk also covers ttyACM-style layouts for free.
for dev in /dev/ttyUSB*; do
    [ -e "$dev" ] || continue
    name=$(basename "$dev")
    syspath="/sys/class/tty/$name/device"
    vid=""
    for _ in 1 2 3 4 5 6; do
        if [ -r "$syspath/idVendor" ]; then
            vid=$(cat "$syspath/idVendor")
            break
        fi
        syspath="$syspath/.."
    done
    if [ "$vid" = "10c4" ]; then
        link "$dev" ttyLIDAR
    else
        echo "  $dev: vendor '${vid:-unknown}' — not the CP2102 LiDAR (skipped)"
    fi
done

echo
echo "result:"
ls -l /dev/ttyMOTOR /dev/ttyLIDAR 2>&1 | sed 's/^/  /'
