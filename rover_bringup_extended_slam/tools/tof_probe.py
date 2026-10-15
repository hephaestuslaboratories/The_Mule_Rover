#!/usr/bin/env python3
"""tof_probe.py — bench bring-up for the 6x VL53L1X ToF belt behind a TCA9548A.

NO ROS.  This is the step BEFORE the ROS node exists: prove the wiring, prove
the mux, prove every sensor answers, and learn which mux channel each physical
sensor is on.  Run it on the Jetson (inside or outside the container — it only
needs /dev/i2c-*).

    # 1. is the mux there at all, and who answers on each channel?
    python3 tof_probe.py --bus 7

    # 2. which channel is which sensor?  (unplug one, re-scan, see what left)
    python3 tof_probe.py --bus 7 --watch

    # 3. live distances — wave a hand, check the floor, confirm aim
    python3 tof_probe.py --bus 7 --live

The belt design (SESSION_HANDOFF.md S14) is 6 sensors, one per mux channel,
all at the factory address 0x29 — the mux is what keeps them apart, so a
channel is the ONLY thing that identifies a sensor.  Get that map right here
and the ROS node is just a loop.

Requires:  pip3 install smbus2
--live also needs:
           pip3 install adafruit-extended-bus \
                        adafruit-circuitpython-vl53l1x \
                        adafruit-circuitpython-tca9548a
"""

import argparse
import sys
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    sys.exit("smbus2 not installed.  pip3 install smbus2")

# VL53L1X identity registers (16-bit register addressing, big endian).
REG_MODEL_ID = 0x010F        # reads 0xEACC over two bytes on a real VL53L1X
EXPECT_MODEL_ID = 0xEACC
SENSOR_ADDR = 0x29           # factory default; we never change it

# Where each sensor lives.  EDIT THIS once --watch tells you the truth.
# Channel -> (label, lens height in metres, max_range cap in metres)
CHANNEL_MAP = {
    0: ("Right_Angle",   0.135, 0.47),
    1: ("Top_Down",  0.135, 0.47),
    2: ("back",  0.155, 0.55),
    3: ("Left_Angle", 0.155, 0.55),
    4: ("Front_Right",         0.365, 1.00),
    7: ("Front_Left",         0.135, 0.47),
}


# ----------------------------------------------------------------- mux helpers

def mux_select(bus, mux_addr, channel):
    """Enable exactly one mux channel (or none, for channel=None).

    Returns None on success, or the OSError if the bus refused the write.
    A timeout here (errno 110) does NOT mean the mux is gone — it means the
    branch we just connected is holding SDA or SCL low, which jams everything.
    """
    try:
        bus.write_byte(mux_addr, 0x00 if channel is None else (1 << channel))
        return None
    except OSError as exc:
        return exc


def mux_present(bus, mux_addr):
    return mux_select(bus, mux_addr, None) is None


def mux_state(bus, mux_addr):
    """Read back the mux control byte — which channels are enabled right now.

    The TCA9548A latches its selection across master resets and script exits,
    so a non-zero value at startup means a previous run left a channel live.
    """
    try:
        return bus.read_byte(mux_addr)
    except OSError:
        return None


WEDGED_HELP = """\
The bus JAMMED the moment channel {ch} was enabled — errno 110 is a line being
held low, not a missing device.  The mux itself is fine.

Recover, then find the culprit:

  1. POWER-CYCLE the mux.  It latches its channel selection, so the bad branch
     stays connected until the 3.3 V is actually removed.  Nothing below works
     until you do this.

  2. UNPLUG the sensor whose power LED is dark, if you have one.  An unpowered
     VL53L1X with SDA/SCL still connected drags the bus down through its
     internal protection diodes.  This is the most common cause by far.

  3. Re-run.  If it is clean now, that sensor (or its cable) is the fault.

  4. Still jammed?  Unplug ALL sensors, power-cycle, and run again: you should
     see the mux answer with every channel silent.  Then plug them back in ONE
     at a time, re-running between each.  The one that jams it is the one.

  5. Meter check with everything unplugged: SDA-to-GND and SCL-to-GND should
     both read ~3.3 V.  A line sitting near 0 V is the one being held down.
"""


def read_model_id(bus):
    """Read the VL53L1X model ID on the currently selected channel.

    Returns the 16-bit value, or None if nothing answered at 0x29.
    """
    try:
        w = i2c_msg.write(SENSOR_ADDR, [REG_MODEL_ID >> 8, REG_MODEL_ID & 0xFF])
        r = i2c_msg.read(SENSOR_ADDR, 2)
        bus.i2c_rdwr(w, r)
        data = list(r)
        return (data[0] << 8) | data[1]
    except OSError:
        return None


def scan(bus, mux_addr, channels):
    """Probe every channel.

    Returns ({channel: model_id or None}, wedged_channel or None).  Stops at
    the first channel that jams the bus, because once a line is held low every
    later result would be a lie.
    """
    found = {}
    for ch in channels:
        if mux_select(bus, mux_addr, ch) is not None:
            return found, ch
        time.sleep(0.01)
        found[ch] = read_model_id(bus)
    mux_select(bus, mux_addr, None)
    return found, None


# --------------------------------------------------------------------- reports

def print_scan(found):
    print(f"  {'ch':>2}  {'label':<14} {'model id':<10} verdict")
    print(f"  {'--':>2}  {'-' * 14} {'-' * 10} {'-' * 30}")
    for ch in sorted(found):
        mid = found[ch]
        label = CHANNEL_MAP.get(ch, ("(unassigned)",))[0]
        if mid is None:
            verdict = "-"
            shown = "-"
        elif mid == EXPECT_MODEL_ID:
            verdict = "OK   VL53L1X"
            shown = f"0x{mid:04X}"
        else:
            verdict = "?    answered, but not a VL53L1X"
            shown = f"0x{mid:04X}"
        print(f"  {ch:>2}  {label:<14} {shown:<10} {verdict}")
    live = [ch for ch, mid in found.items() if mid == EXPECT_MODEL_ID]
    print(f"\n  {len(live)} sensor(s) answering: channels {live}")
    return live


def diagnose(found, expected):
    """Translate a scan result into the wiring fault that caused it."""
    live = [ch for ch, mid in found.items() if mid == EXPECT_MODEL_ID]
    notes = []
    if not live:
        notes.append(
            "NOTHING answered on any channel.  The mux is alive (it ACKed), so "
            "suspect the SENSOR side: VIN/GND swapped, or SDA/SCL swapped, or "
            "the channel pins wired to the wrong column.")
    if len(live) == 8 and expected < 8:
        notes.append(
            "EVERY channel answers but you only wired "
            f"{expected}.  That means the channels are shorted together — "
            "most likely one sensor's SDA/SCL is on the trunk side of the mux "
            "instead of a channel, or two columns are bridged by solder.")
    missing = [ch for ch in CHANNEL_MAP if ch not in live]
    if missing and live:
        labels = ", ".join(f"{ch}={CHANNEL_MAP[ch][0]}" for ch in missing)
        notes.append(
            f"Expected-but-silent channels: {labels}.  For each one check, in "
            "this order: (1) VIN and GND at the sensor with a meter — 3.3 V, "
            "correct polarity; (2) SDA and SCL not swapped; (3) the connector "
            "is fully seated; (4) swap that sensor with a known-good one to "
            "tell a wiring fault from a dead board.")
    return notes


# ------------------------------------------------------------------ live ranging

def live(bus_num, mux_addr, channels, hz):
    try:
        from adafruit_extended_bus import ExtendedI2C
        import adafruit_tca9548a
        import adafruit_vl53l1x
    except ImportError:
        sys.exit(
            "--live needs the Adafruit drivers:\n"
            "  pip3 install adafruit-extended-bus "
            "adafruit-circuitpython-vl53l1x adafruit-circuitpython-tca9548a")

    i2c = ExtendedI2C(bus_num)
    tca = adafruit_tca9548a.TCA9548A(i2c, address=mux_addr)

    sensors = {}
    for ch in channels:
        try:
            s = adafruit_vl53l1x.VL53L1X(tca[ch])
            # Short mode: max ~1.3 m but far more immune to ambient light than
            # long mode, and everything this belt cares about is under 0.6 m.
            s.distance_mode = 1
            s.timing_budget = 33
            s.start_ranging()          # free-running: we only read, never wait
            sensors[ch] = s
        except Exception as exc:                      # noqa: BLE001
            print(f"  channel {ch}: init failed ({exc})")

    if not sensors:
        sys.exit("No sensors initialised — run a plain scan first.")

    labels = [CHANNEL_MAP.get(ch, (f"ch{ch}",))[0] for ch in sorted(sensors)]
    print("\n  Live ranging (cm).  Ctrl-C to stop.")
    print("  Wave a hand in front of ONE sensor to confirm its channel.\n")
    print("  " + "  ".join(f"{lab:>13}" for lab in labels))

    period = 1.0 / hz
    try:
        while True:
            row = []
            for ch in sorted(sensors):
                try:
                    d = sensors[ch].distance          # cm, or None if not ready
                    row.append(f"{d:>13.1f}" if d is not None else f"{'--':>13}")
                except Exception:                     # noqa: BLE001
                    row.append(f"{'ERR':>13}")
            print("  " + "  ".join(row), end="\r", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n")
    finally:
        for s in sensors.values():
            try:
                s.stop_ranging()
            except Exception:                         # noqa: BLE001
                pass


# ------------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=7,
                    help="I2C bus number.  Header pins 3/5 are usually 7 on an "
                         "Orin Nano — confirm with `sudo i2cdetect -l` (default: 7)")
    ap.add_argument("--mux", type=lambda s: int(s, 0), default=0x70,
                    help="TCA9548A address (default: 0x70)")
    ap.add_argument("--channels", default="0-7",
                    help="channels to probe, e.g. 0-7 or 0,1,2,5 (default: 0-7)")
    ap.add_argument("--watch", action="store_true",
                    help="re-scan every second — unplug a sensor and watch which "
                         "channel drops out to build the channel map")
    ap.add_argument("--live", action="store_true",
                    help="continuous distance readout (needs the Adafruit drivers)")
    ap.add_argument("--hz", type=float, default=5.0,
                    help="--live refresh rate (default: 5)")
    args = ap.parse_args()

    if "-" in args.channels:
        lo, hi = args.channels.split("-")
        channels = list(range(int(lo), int(hi) + 1))
    else:
        channels = [int(c) for c in args.channels.split(",")]

    try:
        bus = SMBus(args.bus)
    except FileNotFoundError:
        sys.exit(f"/dev/i2c-{args.bus} does not exist.  Check `sudo i2cdetect -l`, "
                 "and if this is inside the container, that the device is mounted.")
    except PermissionError:
        sys.exit(f"No permission on /dev/i2c-{args.bus}.  Try sudo, or add the "
                 "udev rule / i2c group membership.")

    with bus:
        print(f"\nBus /dev/i2c-{args.bus}, mux at 0x{args.mux:02X}\n")

        state = mux_state(bus, args.mux)
        if state is None:
            sys.exit(
                f"No answer from 0x{args.mux:02X}.  If `i2cdetect -y -r {args.bus}` "
                "DOES show it, the bus is jammed by a branch a previous run left\n"
                "enabled — power-cycle the mux and try again.  Otherwise:\n"
                "  * is the mux powered?  (3.3 V and GND from header pins 1 and 6)\n"
                "  * are SDA and SCL swapped on the trunk?  (pins 3 and 5)\n"
                "  * do the A0/A1/A2 pads move it off 0x70?")
        if state:
            live_chs = [c for c in range(8) if state & (1 << c)]
            print(f"  note: channel(s) {live_chs} were already enabled from an "
                  f"earlier run (control byte 0x{state:02X}).\n")

        if not mux_present(bus, args.mux):
            sys.exit(f"Mux answered a read but refused a write — the bus is held "
                     "low.  Power-cycle the mux and unplug the sensors.")
        print(f"Mux answers at 0x{args.mux:02X}.  Probing channels...\n")

        if args.live:
            found, wedged = scan(bus, args.mux, channels)
            print_scan(found)
            if wedged is not None:
                sys.exit("\n" + WEDGED_HELP.format(ch=wedged))
            wired = [ch for ch, mid in found.items() if mid == EXPECT_MODEL_ID]
            bus.close()
            live(args.bus, args.mux, wired, args.hz)
            return

        if args.watch:
            print("Watching.  Unplug one sensor at a time and note which channel\n"
                  "disappears — that is your channel map.  Ctrl-C to stop.\n")
            last = None
            try:
                while True:
                    found, wedged = scan(bus, args.mux, channels)
                    if wedged is not None:
                        print("\n" + WEDGED_HELP.format(ch=wedged))
                        return
                    live_now = tuple(sorted(
                        ch for ch, mid in found.items() if mid == EXPECT_MODEL_ID))
                    if live_now != last:
                        stamp = time.strftime("%H:%M:%S")
                        print(f"  [{stamp}] answering: {list(live_now)}")
                        last = live_now
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print()
            return

        found, wedged = scan(bus, args.mux, channels)
        print_scan(found)
        if wedged is not None:
            sys.exit("\n" + WEDGED_HELP.format(ch=wedged))
        for note in diagnose(found, len(CHANNEL_MAP)):
            print(f"\n  ! {note}")
        print()


if __name__ == "__main__":
    main()
