#!/usr/bin/env bash
# wifi_link_logger.sh — HOST-side WiFi flight recorder.
#
# WHY: SSH dies while the Jetson stays up, and no log says why. power_save is
# confirmed off, so something else is dropping the link. This writes one line
# every 2 s and fsyncs it, so the evidence survives a hard power-off.
#
# Each line: time | operstate | SSID / BSSID / signal / tx bitrate | IP | gateway ping
#
# Reading it after an incident — find the moment `gw=` goes to FAIL:
#   - state=down or "not connected"    -> the Jetson lost association (driver/AP)
#   - associated, signal ok, gw FAIL   -> link up but no traffic (AP/ARP/driver stall)
#   - no ip=                           -> DHCP lease lost
#   - everything OK the whole time     -> the Jetson was reachable; look at the PC side
#   - BSSID changes                    -> it roamed to another AP/band
#
# Install: see wifi_link_logger.service

IFACE="${1:-wlP1p1s0}"
LOG=/var/log/wifi_link.log
MAX_BYTES=$((20 * 1024 * 1024))

while true; do
    if [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt "$MAX_BYTES" ]; then
        mv -f "$LOG" "$LOG.1"
    fi

    state=$(cat "/sys/class/net/$IFACE/operstate" 2>/dev/null || echo missing)
    link=$(iw dev "$IFACE" link 2>&1 | awk '
        /Not connected/      {print "not-connected"; exit}
        /^Connected to/      {printf "bssid=%s ", $3}
        /SSID:/              {printf "ssid=%s ", $2}
        /signal:/            {printf "sig=%s ", $2}
        /tx bitrate:/        {printf "tx=%s ", $3}')
    ip4=$(ip -4 -o addr show dev "$IFACE" 2>/dev/null | awk '{print $4}')
    gw=$(ip route | awk '/^default/ {print $3; exit}')
    if [ -n "$gw" ] && out=$(ping -c1 -W1 -I "$IFACE" "$gw" 2>/dev/null); then
        gwres="OK $(echo "$out" | sed -n 's/.*time=\([0-9.]*\).*/\1ms/p')"
    else
        gwres="FAIL"
    fi

    echo "$(date '+%F %T') state=$state $link ip=${ip4:-none} gw=${gw:-none} $gwres" >> "$LOG"
    sync -d "$LOG"
    sleep 2
done
