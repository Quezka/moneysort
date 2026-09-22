#!/bin/bash
# Show the Money Sorter dashboard fullscreen on the Pi's display.
#
# Prefers cog (WPE WebKit) -- much lighter than Chromium -- and falls back to
# Chromium if cog isn't installed, so the switch is safe. Install cog once:
#     sudo apt install -y cog
#
# The dashboard is served by the armd systemd service (moneysort-arm); this only
# waits for it to come up, then launches the browser. Run inside the Pi's desktop
# session or via ~/.config/labwc/autostart. Does NOT start a server.
URL=http://localhost:8080

# Wait (up to ~30s) for armd to be serving.
for i in $(seq 1 30); do
    curl -s "$URL/status" >/dev/null 2>&1 && break
    sleep 1
done

if command -v cog >/dev/null 2>&1; then
    # cog is a kiosk browser: one view, no chrome. Its Wayland platform plugin
    # has no fullscreen switch (-O fullscreen=true is silently ignored), so the
    # compositor does it: deploy/setup.sh adds a labwc window rule
    #   <windowRule identifier="com.igalia.Cog"><action name="ToggleFullscreen"/>
    # that fullscreens cog when its window maps (identifier = cog's app-id).
    exec cog "$URL"
else
    exec chromium \
        --kiosk --app="$URL" \
        --ozone-platform=wayland \
        --password-store=basic \
        --noerrdialogs --disable-infobars --incognito \
        --disable-features=Translate --check-for-update-interval=31536000
fi
