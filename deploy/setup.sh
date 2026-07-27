#!/bin/bash
# Install / refresh the Money Sorter arm daemon (armd) as a systemd service.
# Run this ON THE PI after deploy.sh has pulled the latest code:
#     bash ~/projects/moneysort/deploy/setup.sh
#
# It replaces the old dashboard-only service (moneysort-dashboard) with the
# arm daemon (moneysort-arm), which owns the hardware and serves the dashboard
# on the same port 8080 -- so the kiosk needs no changes.
set -e
DIR="/home/money-sorter/projects/moneysort"

echo "Installing moneysort-arm.service ..."
sudo cp "$DIR/deploy/moneysort-arm.service" /etc/systemd/system/moneysort-arm.service
sudo systemctl daemon-reload

# Retire the old dashboard-only service if it exists (armd replaces it).
if systemctl list-unit-files | grep -q '^moneysort-dashboard\.service'; then
    echo "Retiring old moneysort-dashboard.service ..."
    sudo systemctl disable --now moneysort-dashboard.service || true
fi

echo "Enabling + starting moneysort-arm.service ..."
sudo systemctl enable --now moneysort-arm.service
sleep 1
systemctl --no-pager --full status moneysort-arm.service | head -6
echo
echo "Done. Dashboard + arm daemon on http://localhost:8080"
