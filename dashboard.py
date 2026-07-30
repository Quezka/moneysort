#!/usr/bin/env python3
"""Money Sorter status dashboard - a dependency-free web UI for the Pi's screen.

Serves a dark, touch-friendly page (sized for the 1280x800 panel) showing the
essentials: system health plus the arm's joint state. Uses only the Python
standard library, so no pip install is needed.

Run:
    python3 dashboard.py            # serves on http://0.0.0.0:8080
Show it fullscreen on the Pi with kiosk.sh, or open the URL from any device.

Arm state is read from state.json (written by the arm controller) if present;
until then the arm tiles show "no data" gracefully.
"""
import json
import os
import socket
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8080
STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "state.json")
ENABLE_PIN = 26   # shared driver enable (active-low): LOW = enabled, HIGH = disabled


# --- metric collection ------------------------------------------------------
def _read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def cpu_temp_c():
    raw = _read("/sys/class/thermal/thermal_zone0/temp", "0")
    try:
        return round(int(raw) / 1000.0, 1)
    except ValueError:
        return None


def load_avg():
    try:
        return float(_read("/proc/loadavg", "0").split()[0])
    except (ValueError, IndexError):
        return None


def mem_pct():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        parts = line.split(":")
        if len(parts) == 2:
            info[parts[0]] = int(parts[1].strip().split()[0])
    total, avail = info.get("MemTotal"), info.get("MemAvailable")
    if total and avail:
        return round((total - avail) / total * 100), round(total / 1_048_576, 1)
    return None, None


def disk_pct():
    try:
        s = os.statvfs("/")
        used = (s.f_blocks - s.f_bfree) * s.f_frsize
        total = s.f_blocks * s.f_frsize
        return round(used / total * 100), round(total / 1_000_000_000, 1)
    except OSError:
        return None, None


def uptime_str():
    try:
        secs = int(float(_read("/proc/uptime", "0").split()[0]))
    except (ValueError, IndexError):
        return "?"
    d, rem = divmod(secs, 86400)
    h, rem = divmod(rem, 3600)
    m = rem // 60
    return (f"{d}d " if d else "") + f"{h}h {m}m"


def ip_addr():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "?"


def arm_state():
    """Read the arm controller's state file if it exists and is fresh."""
    try:
        with open(STATE_PATH) as f:
            data = json.load(f)
        age = time.time() - os.path.getmtime(STATE_PATH)
        data["_age"] = round(age, 1)
        data["_live"] = age < 5
        return data
    except (OSError, ValueError):
        return None


def motors_enabled():
    """Real level of the shared enable pin (active-low): True if motors are on."""
    try:
        out = subprocess.run(["pinctrl", "get", str(ENABLE_PIN)],
                             capture_output=True, text=True, timeout=1).stdout
        seg = out.split("|")
        if len(seg) >= 2:
            return seg[1].strip().split()[0] == "lo"   # low = enabled
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def set_enable(enable):
    """Drive the shared enable pin. True -> motors enabled; False -> disabled."""
    level = "dl" if enable else "dh"
    try:
        subprocess.run(["pinctrl", "set", str(ENABLE_PIN), "op", level],
                       timeout=2, check=True)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def system_action(action):
    """Reboot or power off the Pi (needs passwordless sudo)."""
    cmd = {"reboot": ["sudo", "reboot"], "poweroff": ["sudo", "poweroff"]}.get(action)
    if not cmd:
        return False
    try:
        subprocess.Popen(cmd)   # fire-and-forget; the box goes down under us
        return True
    except OSError:
        return False


def exit_kiosk():
    """Close the fullscreen Chromium kiosk, returning to the labwc desktop."""
    try:
        subprocess.Popen(["pkill", "-f", "chromium"])
        return True
    except OSError:
        return False


def snapshot():
    mem_used, mem_total = mem_pct()
    disk_used, disk_total = disk_pct()
    return {
        "host": socket.gethostname(),
        "ip": ip_addr(),
        "temp": cpu_temp_c(),
        "load": load_avg(),
        "mem_pct": mem_used, "mem_total": mem_total,
        "disk_pct": disk_used, "disk_total": disk_total,
        "uptime": uptime_str(),
        "motors_enabled": motors_enabled(),
        "arm": arm_state(),
        "time": time.strftime("%H:%M:%S"),
    }


# --- web server -------------------------------------------------------------
PAGE = """<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Money Sorter</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: system-ui, sans-serif; background: #0e1116; color: #e6edf3;
         height: 100vh; overflow: hidden; padding: 18px; }
  header { display: flex; align-items: baseline; justify-content: space-between; margin-bottom: 14px; }
  h1 { font-size: 26px; font-weight: 700; letter-spacing: .5px; }
  h1 span { color: #58a6ff; }
  .clock { font-size: 20px; color: #8b949e; font-variant-numeric: tabular-nums; }
  .grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 14px; }
  .card { background: #161b22; border: 1px solid #21262d; border-radius: 14px; padding: 16px 18px; }
  .label { font-size: 13px; text-transform: uppercase; letter-spacing: 1px; color: #8b949e; margin-bottom: 8px; }
  .value { font-size: 34px; font-weight: 700; font-variant-numeric: tabular-nums; }
  .unit { font-size: 18px; color: #8b949e; font-weight: 500; }
  .sub { font-size: 13px; color: #6e7681; margin-top: 4px; }
  .ok { color: #3fb950; } .warn { color: #d29922; } .bad { color: #f85149; }
  .section { font-size: 15px; text-transform: uppercase; letter-spacing: 1.5px;
             color: #8b949e; margin: 20px 0 12px; display: flex; align-items: center; gap: 10px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: #6e7681; }
  .dot.live { background: #3fb950; box-shadow: 0 0 8px #3fb950; }
  .joints { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
  .joint .value { font-size: 42px; }
  .moving { color: #58a6ff; } .idle { color: #6e7681; }
  .controls { display: flex; align-items: center; gap: 14px; margin-top: 22px; }
  .spacer { flex: 1; }
  button { font-family: inherit; font-weight: 700; border: none; border-radius: 12px;
           cursor: pointer; color: #fff; -webkit-tap-highlight-color: transparent; }
  button:active { transform: translateY(3px); box-shadow: none !important; }
  .estop { background: #da3633; font-size: 22px; letter-spacing: 1px; padding: 20px 34px; box-shadow: 0 4px 0 #a5201d; }
  .estop.off { background: #3d1d1d; color: #f85149; box-shadow: 0 4px 0 #2a1414; }
  .reenable { background: #238636; font-size: 16px; padding: 16px 22px; box-shadow: 0 4px 0 #196c2b; }
  .sys { background: #30363d; font-size: 16px; padding: 16px 22px; box-shadow: 0 4px 0 #21262d; }
  .sys.danger { background: #6e2b2b; box-shadow: 0 4px 0 #4a1d1d; }
</style></head><body>
  <header>
    <h1>Money<span>Sorter</span></h1>
    <div class="clock" id="clock">--:--:--</div>
  </header>

  <div class="grid" id="sys"></div>

  <div class="section"><span class="dot" id="armdot"></span>Robot Arm <span id="armstate" class="idle" style="font-size:13px"></span></div>
  <div class="joints" id="joints"></div>

  <div class="controls">
    <button id="estop" class="estop">&#9940; EMERGENCY DISABLE</button>
    <button id="reenable" class="reenable" style="display:none">Re-enable motors</button>
    <button id="zero" class="sys">&#9678; Zero position</button>
    <button id="home" class="sys">&#8962; Home</button>
    <span class="spacer"></span>
    <button id="desktop" class="sys">&#128421; Desktop</button>
    <button id="reboot" class="sys">&#8635; Reboot</button>
    <button id="poweroff" class="sys danger">&#9099; Power off</button>
  </div>

<script>
const cls = (v, warn, bad) => v == null ? "" : v >= bad ? "bad" : v >= warn ? "warn" : "ok";
const card = (label, value, unit, sub, klass="") =>
  `<div class="card"><div class="label">${label}</div>
   <div class="value ${klass}">${value ?? "&mdash;"}<span class="unit">${unit||""}</span></div>
   <div class="sub">${sub||""}</div></div>`;

async function tick() {
  let d; try { d = await (await fetch("/status")).json(); } catch { return; }
  document.getElementById("clock").textContent = d.time;

  document.getElementById("sys").innerHTML =
    card("CPU Temp", d.temp, "&deg;C", "", cls(d.temp, 65, 80)) +
    card("CPU Load", d.load, "", "1-min average") +
    card("Memory", d.mem_pct, "%", (d.mem_total||"?")+" GB total", cls(d.mem_pct, 75, 90)) +
    card("Disk", d.disk_pct, "%", (d.disk_total||"?")+" GB total", cls(d.disk_pct, 80, 92)) +
    card("Host", d.host, "", d.ip) +
    card("Uptime", d.uptime, "", "");

  const a = d.arm, dot = document.getElementById("armdot"), st = document.getElementById("armstate");
  const names = ["x", "y", "z"];
  if (a) {
    dot.className = a.moving ? "dot live" : "dot";
    const age = a._age != null ? `updated ${a._age}s ago` : "";
    st.textContent = a.moving ? "MOVING" : ("idle · " + age);
    st.className = a.moving ? "moving" : "idle";
    document.getElementById("joints").innerHTML = names.map(n =>
      `<div class="card joint"><div class="label">${n}</div>
       <div class="value">${a.joints && a.joints[n]!=null ? a.joints[n] : "&mdash;"}<span class="unit">&deg;</span></div></div>`
    ).join("");
  } else {
    dot.className = "dot";
    st.textContent = "no data yet";
    document.getElementById("joints").innerHTML = names.map(n =>
      `<div class="card joint"><div class="label">${n}</div><div class="value idle">&mdash;<span class="unit">&deg;</span></div></div>`
    ).join("");
  }

  const en = d.motors_enabled, estop = document.getElementById("estop"), reen = document.getElementById("reenable");
  if (en === false) { estop.textContent = "MOTORS DISABLED"; estop.classList.add("off"); reen.style.display = ""; }
  else if (en === true) { estop.innerHTML = "&#9940; EMERGENCY DISABLE"; estop.classList.remove("off"); reen.style.display = "none"; }
}

async function post(path, body) {
  try {
    await fetch(path, {
      method: "POST",
      headers: body ? { "Content-Type": "application/json" } : {},
      body: body ? JSON.stringify(body) : undefined,
    });
  } catch {}
}
document.getElementById("estop").onclick = () => post("/disable").then(tick);
document.getElementById("reenable").onclick = () => post("/enable").then(tick);
document.getElementById("zero").onclick = () => { if (confirm("Set current position as zero?")) post("/zero").then(tick); };
document.getElementById("home").onclick = () => { if (confirm("Home all axes? X and Y seek their switches; Z returns to zero.")) post("/home").then(tick); };
document.getElementById("desktop").onclick = () => post("/kiosk-exit");
document.getElementById("reboot").onclick = () => { if (confirm("Reboot the Pi?")) post("/reboot"); };
document.getElementById("poweroff").onclick = () => { if (confirm("Power OFF the Pi?")) post("/poweroff"); };
tick(); setInterval(tick, 1500);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        pass

    def _send(self, body, ctype):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/status"):
            self._send(json.dumps(snapshot()), "application/json")
        else:
            self._send(PAGE, "text/html; charset=utf-8")

    def do_POST(self):
        actions = {
            "/disable":  lambda: set_enable(False),   # emergency: cut motor torque
            "/enable":   lambda: set_enable(True),
            "/reboot":   lambda: system_action("reboot"),
            "/poweroff": lambda: system_action("poweroff"),
        }
        fn = next((f for p, f in actions.items() if self.path.startswith(p)), None)
        if fn is None:
            self.send_response(404)
            self.end_headers()
            return
        ok = fn()
        self._send(json.dumps({"ok": ok, "motors_enabled": motors_enabled()}),
                   "application/json")


def main():
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Money Sorter dashboard: http://{ip_addr()}:{PORT}  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
