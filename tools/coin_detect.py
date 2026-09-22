#!/usr/bin/env python3
"""Ask armd to detect coins and save the annotated frame (thin HTTP client).

Run ON THE PI (armd owns the camera now, so this talks to it over HTTP):
    python3 ~/projects/moneysort/tools/coin_detect.py [out_dir]

Prints each detected coin (camera-frame 3D, mm) from POST /detect and saves the
annotated image from GET /detected. To view live, just open the dashboard --
this is for a quick CLI check and for saving a frame to scp.
"""
import json
import os
import sys
import urllib.request

ARMD = "http://localhost:8080"


def get_bytes(path):
    with urllib.request.urlopen(ARMD + path, timeout=10) as r:
        return r.read(), r.headers.get_content_type()


def post_json(path, body=None):
    req = urllib.request.Request(ARMD + path, data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)
    try:
        res = post_json("/detect")
    except Exception as e:
        sys.exit(f"can't reach armd /detect ({e}); is the service up + camera ok?")

    coins = res.get("coins", [])
    print(f"detected {len(coins)} coin(s):")
    for c in coins:
        x, y, z = c["cam_mm"]
        print(f"  px=({c['px']:4},{c['py']:4}) r={c['r']:2}  depth={c['depth_m']:.3f}m  "
              f"cam-frame xyz=({x:+.0f},{y:+.0f},{z:+.0f}) mm")

    try:
        data, ctype = get_bytes("/detected")
        if ctype == "image/jpeg":
            path = os.path.join(out_dir, "coins_annotated.jpg")
            with open(path, "wb") as f:
                f.write(data)
            print(f"\nsaved annotated frame -> {path}  (scp/view it)")
        else:
            print("\n(no annotated image:", data.decode(errors="ignore")[:120], ")")
    except Exception as e:
        print(f"\n(couldn't fetch /detected: {e})")


if __name__ == "__main__":
    main()
