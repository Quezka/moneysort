#!/bin/bash
# Show the Money Sorter dashboard fullscreen on the Pi's display.
#
# The dashboard is served by the armd systemd service (moneysort-arm), so this
# script does NOT start a server -- it only waits for armd to come up, then
# launches Chromium in kiosk mode. Starting a second server here is what caused
# a port-8080 clash with armd, so don't reintroduce it.
#
# Run inside the Pi's desktop session, or via ~/.config/labwc/autostart.

# Wait (up to ~30s) for armd to be serving.
for i in $(seq 1 30); do
    curl -s http://localhost:8080/status >/dev/null 2>&1 && break
    sleep 1
done

exec chromium \
    --kiosk --app=http://localhost:8080 \
    --ozone-platform=wayland \
    --password-store=basic \
    --noerrdialogs --disable-infobars --incognito \
    --disable-features=Translate --check-for-update-interval=31536000
