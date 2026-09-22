#!/bin/bash
# Install / refresh the Money Sorter arm daemon (armd) as a systemd service,
# and install the desktop launcher used to re-open the kiosk after "Desktop".
# Run ON THE PI after deploy.sh has pulled the latest code:
#     bash ~/projects/moneysort/deploy/setup.sh
#
# armd owns the hardware AND serves the dashboard on port 8080, so the kiosk
# needs no changes. This script is safe to re-run.
set -e
DIR="/home/money-sorter/projects/moneysort"

echo "Installing moneysort-arm.service ..."
sudo cp "$DIR/deploy/moneysort-arm.service" /etc/systemd/system/moneysort-arm.service
sudo systemctl daemon-reload

# Retire the old dashboard-only service if present (armd replaces it).
if systemctl list-unit-files | grep -q '^moneysort-dashboard\.service'; then
    echo "Retiring old moneysort-dashboard.service ..."
    sudo systemctl disable --now moneysort-dashboard.service || true
fi

# Kill any stray dashboard.py (e.g. started by an old kiosk.sh) holding :8080 --
# it would stop armd from binding and cause a crash-restart loop.
if pkill -f 'dashboard\.py'; then echo "Killed stray dashboard.py"; fi
sleep 1

echo "Enabling + (re)starting moneysort-arm.service ..."
sudo systemctl enable moneysort-arm.service
sudo systemctl restart moneysort-arm.service
sleep 1

# Ensure labwc autostart launches the kiosk via `bash` (git doesn't preserve the
# exec bit, so invoking the script path directly can fail with permission denied).
echo "Configuring labwc autostart ..."
AUTOSTART="$HOME/.config/labwc/autostart"
install -d "$HOME/.config/labwc"
touch "$AUTOSTART"
grep -v 'kiosk\.sh' "$AUTOSTART" > "$AUTOSTART.tmp" 2>/dev/null || true
mv "$AUTOSTART.tmp" "$AUTOSTART"
echo "bash $DIR/kiosk.sh &" >> "$AUTOSTART"

# Make labwc fullscreen the cog kiosk. cog's Wayland platform has no fullscreen
# switch, so the compositor does it via a window rule matching cog's app-id
# (com.igalia.Cog). Merge the rule into rc.xml, preserving any existing content
# (e.g. the touchscreen mapping); idempotent. NOTE: window rules apply when a
# window MAPS, and restarting cog alone doesn't reload labwc -- so this reloads
# labwc, and the kiosk shows fullscreen on its next launch (reboot, or re-open
# via the Desktop launcher).
echo "Ensuring labwc fullscreens the cog kiosk ..."
RC="$HOME/.config/labwc/rc.xml"
python3 - "$RC" <<'PY'
import os, sys
rc = sys.argv[1]
RULE = ("  <windowRules>\n"
        "    <!-- Money Sorter kiosk: fullscreen the cog browser when it maps -->\n"
        "    <windowRule identifier=\"com.igalia.Cog\">\n"
        "      <action name=\"ToggleFullscreen\"/>\n"
        "    </windowRule>\n"
        "  </windowRules>\n")
skeleton = ('<?xml version="1.0"?>\n'
            '<openbox_config xmlns="http://openbox.org/3.4/rc">\n'
            '</openbox_config>\n')
s = open(rc).read() if os.path.exists(rc) else skeleton
if "com.igalia.Cog" in s:
    print("  rc.xml already has the cog fullscreen rule")
elif "</openbox_config>" in s:
    os.makedirs(os.path.dirname(rc), exist_ok=True)
    open(rc, "w").write(s.replace("</openbox_config>", RULE + "</openbox_config>"))
    print("  added cog fullscreen windowRule to rc.xml")
else:                                   # unrecognisable rc.xml -- don't clobber it
    print("  WARNING: rc.xml has no </openbox_config>; add the windowRule by hand")
PY
killall -SIGHUP labwc 2>/dev/null && echo "  reloaded labwc config" || true

# Desktop + menu launcher to re-open the kiosk after "Exit to Desktop".
echo "Installing desktop launcher ..."
install -d "$HOME/Desktop" "$HOME/.local/share/applications"
cp "$DIR/deploy/moneysort-dashboard.desktop" "$HOME/Desktop/"
cp "$DIR/deploy/moneysort-dashboard.desktop" "$HOME/.local/share/applications/"
chmod +x "$HOME/Desktop/moneysort-dashboard.desktop"
gio set "$HOME/Desktop/moneysort-dashboard.desktop" metadata::trusted true 2>/dev/null || true

echo
systemctl --no-pager --full status moneysort-arm.service | head -6
echo
echo "Done. Dashboard + arm daemon on http://localhost:8080"
