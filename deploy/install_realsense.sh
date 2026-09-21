#!/bin/bash
# Install pyrealsense2 on the Raspberry Pi for the Money Sorter camera (D435).
#
# Run ON THE PI:
#     bash ~/projects/moneysort/deploy/install_realsense.sh
#
# It first tries a prebuilt pip wheel; if none exists for this Python / arch
# (likely on the Pi's Python 3.13 / aarch64), it builds librealsense from source
# with the Python bindings. The source build takes ~30-90 min on a Pi 4 -- start
# it and let it run.
#
# Installs into the SYSTEM python3 (the arm daemon runs on /usr/bin/python3), so
# the camera code imports pyrealsense2 from the same place armd runs.
#
# Notes:
#  - Uses FORCE_RSUSB_BACKEND (user-space USB) so no kernel patching is needed.
#  - If `make` gets Killed (out of memory), add swap or drop to `make -j1`.
#  - Plug the D435 into a USB3 (blue) port with a USB3 cable; re-plug after the
#    udev rules step.
set -e

PY=/usr/bin/python3
JOBS=2                     # parallel build jobs; lower to 1 if the Pi OOMs

ok() { $PY -c "import pyrealsense2 as rs; print('pyrealsense2', rs.__version__)"; }

echo "== 0. Already installed? =="
if ok 2>/dev/null; then echo "pyrealsense2 already importable -- nothing to do."; exit 0; fi

echo "== 1. Try a prebuilt wheel =="
if sudo $PY -m pip install --break-system-packages pyrealsense2 2>/dev/null && ok 2>/dev/null; then
    echo "Installed pyrealsense2 from a wheel -- done."
    exit 0
fi
echo "No usable wheel; building librealsense from source."

echo "== 2. Build dependencies (apt) =="
sudo apt-get update
sudo apt-get install -y git cmake build-essential pkg-config \
    libssl-dev libusb-1.0-0-dev libudev-dev python3-dev

SRC="$HOME/librealsense"
echo "== 3. Clone librealsense -> $SRC =="
if [ ! -d "$SRC/.git" ]; then
    git clone --depth 1 https://github.com/IntelRealSense/librealsense.git "$SRC"
fi
cd "$SRC"

echo "== 4. Install udev rules (non-root device access) =="
sudo ./scripts/setup_udev_rules.sh || true

echo "== 5. Configure + build (the long part; make -j$JOBS) =="
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DPYTHON_EXECUTABLE="$PY"
make -j"$JOBS"
sudo make install
sudo ldconfig

echo "== 6. Make the Python bindings importable by system python3 =="
SITE=$($PY -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
echo "  copying pyrealsense2*.so -> $SITE"
sudo find "$SRC/build" -name 'pyrealsense2*.so' -exec cp {} "$SITE"/ \;
sudo ldconfig

echo "== 7. Verify =="
if ok; then
    echo
    echo "SUCCESS. Now test the camera:  python3 ~/projects/moneysort/tools/realsense_test.py"
    echo "(If it can't find the device, re-plug the D435 into a USB3 port.)"
else
    echo "Build finished but 'import pyrealsense2' failed -- check the output above."
    exit 1
fi
