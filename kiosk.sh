#!/bin/bash
# Show the Money Sorter dashboard fullscreen on the Pi's display.
# Run this INSIDE the Pi's desktop session (not over plain SSH), or add it to
# ~/.config/labwc/autostart to launch on boot.
#
# It starts the dashboard server (if not already running) then opens Chromium
# in kiosk mode pointing at it.

DIR="$(cd "$(dirname "$0")" && pwd)"

# start the server in the background if the port is closed
if ! curl -s http://localhost:8080/status >/dev/null 2>&1; then
    python3 "$DIR/dashboard.py" &
    sleep 1
fi

exec chromium \
    --kiosk --app=http://localhost:8080 \
    --ozone-platform=wayland \
    --noerrdialogs --disable-infobars --incognito \
    --disable-features=Translate --check-for-update-interval=31536000
