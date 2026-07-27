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
