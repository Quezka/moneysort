#!/bin/bash
# Pull latest code and restart the arm daemon so changes take effect.
#
# This is the canonical deploy script; deploy/setup.sh installs a copy to
# ~/deploy.sh so the familiar `bash ~/deploy.sh` now also restarts moneysort-arm
# (a plain git pull isn't enough -- the daemon holds the old code in memory).
set -e
cd /home/money-sorter/projects/moneysort
git pull
source .env/bin/activate
pip install -r requirements.txt
sudo systemctl restart moneysort-arm 2>/dev/null && echo "restarted moneysort-arm" || true
echo "deploy complete"
