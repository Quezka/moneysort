#!/usr/bin/env python3
"""Dashboard presentation: the HTML PAGE plus system-metric helpers.

Pure presentation/telemetry for the Money Sorter web UI, dependency-free (Python
standard library only). The armd server (``interface/server.py``) imports
``PAGE`` and these helpers, owns the arm, and assembles ``/status``. This module
touches no GPIO and runs no server of its own.
"""
import os
import socket
import subprocess


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


# --- page -------------------------------------------------------------------
PAGE = """<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Money Sorter</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: system-ui, sans-serif; background: #0e1116; color: #e6edf3;
         height: 100vh; overflow: hidden; padding: 10px; }
  header { display: flex; align-items: baseline; justify-content: space-between; margin-bottom: 8px; }
  h1 { font-size: 22px; font-weight: 700; letter-spacing: .5px; }
  h1 span { color: #58a6ff; }
  .clock { font-size: 18px; color: #8b949e; font-variant-numeric: tabular-nums; }
  .grid { display: grid; grid-template-columns: repeat(6, 1fr); gap: 10px; }
  .card { background: #161b22; border: 1px solid #21262d; border-radius: 12px; padding: 8px 11px; }
  .label { font-size: 11px; text-transform: uppercase; letter-spacing: 1px; color: #8b949e; margin-bottom: 2px; }
  .value { font-size: 22px; font-weight: 700; font-variant-numeric: tabular-nums; }
  .unit { font-size: 14px; color: #8b949e; font-weight: 500; }
  .sub { font-size: 11px; color: #6e7681; margin-top: 1px; }
  .ok { color: #3fb950; } .warn { color: #d29922; } .bad { color: #f85149; }
  .section { font-size: 14px; text-transform: uppercase; letter-spacing: 1.5px;
             color: #8b949e; margin: 10px 0 6px; display: flex; align-items: center; gap: 10px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: #6e7681; }
  .dot.live { background: #3fb950; box-shadow: 0 0 8px #3fb950; }
  .joints { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
  .cammain { display: flex; gap: 14px; align-items: flex-start; margin-top: 4px; flex-wrap: wrap; }
  .camcol { flex: 0 0 auto; }
  .ctrlcol { flex: 0 0 220px; display: flex; flex-direction: column; gap: 8px; }
  .controls.vert { flex-direction: column; align-items: stretch; margin-top: 0; gap: 8px; }
  .controls.vert button { width: 100%; }
  .controls.vert .estop { font-size: 15px; padding: 11px 12px; letter-spacing: .5px; }
  .controls.vert .sys, .controls.vert .reenable { font-size: 14px; padding: 9px 12px; }
  .camwrap { position: relative; background: #0d1117; border: 1px solid #21262d;
             border-radius: 14px; overflow: hidden; aspect-ratio: 1/1; width: 260px; max-width: 42vw; }
  .cam { width: 100%; height: 100%; object-fit: contain; display: block; }
  .camoff { position: absolute; inset: 0; display: flex; align-items: center;
            justify-content: center; color: #6e7681; font-size: 15px; }
  .joint .value { font-size: 26px; }
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
  .sys.go { background: #1f6feb; box-shadow: 0 4px 0 #164a9e; }
  .settings { margin-top: 2px; border-top: 1px solid #21262d; padding-top: 4px; }
  .settings > summary { list-style: none; cursor: pointer; color: #8b949e; font-size: 13px;
             text-transform: uppercase; letter-spacing: 1.5px; padding: 8px 0; }
  .settings > summary::-webkit-details-marker { display: none; }
  .settings[open] > summary { color: #c9d1d9; }
  .settings .controls { margin-top: 8px; }
  #toasts { position: fixed; top: 24px; left: 50%; transform: translateX(-50%);
            display: flex; flex-direction: column; align-items: center; gap: 12px;
            z-index: 100; pointer-events: none; width: max-content; max-width: 92vw; }
  .toast { min-width: 320px; max-width: 620px; background: #1b2230; border: 1px solid #30363d;
           border-left: 7px solid #58a6ff; border-radius: 13px; padding: 19px 28px; color: #f0f3f6;
           font-size: 21px; font-weight: 700; letter-spacing: .2px; box-shadow: 0 16px 44px rgba(0,0,0,.6);
           display: flex; align-items: center; gap: 15px;
           opacity: 0; transform: translateY(-18px) scale(.95);
           transition: opacity .22s ease, transform .22s cubic-bezier(.2,.9,.3,1.3); }
  .toast.show { opacity: 1; transform: none; }
  .toast .ico { font-size: 26px; line-height: 1; }
  .toast.info    { border-left-color: #58a6ff; background: #172234; } .toast.info    .ico { color: #58a6ff; }
  .toast.success { border-left-color: #3fb950; background: #15271a; } .toast.success .ico { color: #3fb950; }
  .toast.warn    { border-left-color: #d29922; background: #2a2413; } .toast.warn    .ico { color: #d29922; }
  .toast.error   { border-left-color: #f85149; background: #2c1919; } .toast.error   .ico { color: #f85149; }
</style></head><body>
  <div id="toasts"></div>
  <header>
    <h1>Money<span>Sorter</span></h1>
    <div class="clock" id="clock">--:--:--</div>
  </header>

  <div class="grid" id="sys"></div>

  <div class="section"><span class="dot" id="armdot"></span>Robot Arm <span id="armstate" class="idle" style="font-size:13px"></span></div>
  <div class="joints" id="joints"></div>

  <div class="cammain">
    <div class="camcol">
      <div class="section"><span class="dot" id="camdot"></span>Camera <span id="camstate" class="idle" style="font-size:13px"></span></div>
      <div class="camwrap">
        <img id="cam" class="cam" alt="camera feed">
        <div id="camoff" class="camoff">camera offline</div>
      </div>
    </div>
    <div class="ctrlcol">
      <div class="controls vert">
        <button id="estop" class="estop">&#9940; EMERGENCY DISABLE</button>
        <button id="reenable" class="reenable" style="display:none">Re-enable motors</button>
        <button id="gozero" class="sys go">&#8617; Return to zero</button>
        <button id="zero" class="sys">&#9678; Set zero here</button>
        <button id="desktop" class="sys">&#128421; Desktop</button>
        <button id="reboot" class="sys">&#8635; Reboot</button>
        <button id="poweroff" class="sys danger">&#9099; Power off</button>
      </div>
      <details class="settings">
        <summary>&#9881; Settings</summary>
        <div class="controls vert">
          <button id="home" class="sys">&#8962; Home axes (seek switches)</button>
        </div>
      </details>
    </div>
  </div>

<script>
const cls = (v, warn, bad) => v == null ? "" : v >= bad ? "bad" : v >= warn ? "warn" : "ok";
const card = (label, value, unit, sub, klass="") =>
  `<div class="card"><div class="label">${label}</div>
   <div class="value ${klass}">${value ?? "&mdash;"}<span class="unit">${unit||""}</span></div>
   <div class="sub">${sub||""}</div></div>`;

let last = {};                       // most recent /status, for click-time checks

const ICON = { info: "&#8505;", success: "&#10003;", warn: "&#9888;", error: "&#10007;" };
function toast(msg, type = "info", ttl = 3800) {
  const wrap = document.getElementById("toasts");
  const el = document.createElement("div");
  el.className = "toast " + type;
  el.innerHTML = `<span class="ico">${ICON[type] || ICON.info}</span><span>${msg}</span>`;
  wrap.appendChild(el);
  requestAnimationFrame(() => el.classList.add("show"));
  setTimeout(() => { el.classList.remove("show"); setTimeout(() => el.remove(), 300); }, ttl);
}

let build = null;                    // daemon build id; reload the page if it changes
async function tick() {
  let d; try { d = await (await fetch("/status")).json(); } catch { return; }
  if (build && d.build && d.build !== build) { location.reload(); return; }
  build = d.build;
  last = d;
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

  const cam = d.camera || {}, cdot = document.getElementById("camdot");
  const cimg = document.getElementById("cam"), coff = document.getElementById("camoff");
  cdot.className = cam.ok ? "dot live" : "dot";
  document.getElementById("camstate").textContent = cam.ok ? (cam.desc || "live") : (cam.error || "offline");
  if (cam.ok) {
    coff.style.display = "none"; cimg.style.display = "";
    if (!cimg.src) cimg.src = "/camera?detect=1";   // annotated MJPEG stream (coins circled)
  } else {
    cimg.style.display = "none"; cimg.removeAttribute("src"); coff.style.display = "";
  }
}

async function post(path, body) {
  try {
    const r = await fetch(path, {
      method: "POST",
      headers: body ? { "Content-Type": "application/json" } : {},
      body: body ? JSON.stringify(body) : undefined,
    });
    let j = {}; try { j = await r.json(); } catch {}
    return { ok: r.ok && j.ok !== false, ...j };
  } catch { return { ok: false, error: "no connection to arm" }; }
}

const atZero = () => last.arm && last.arm.joints &&
  Object.values(last.arm.joints).every(v => v === 0);

document.getElementById("estop").onclick = async () => {
  await post("/disable");
  toast("Emergency stop — motors disabled", "warn");
  tick();
};
document.getElementById("reenable").onclick = async () => {
  const r = await post("/enable");
  toast(r.ok ? "Motors re-enabled" : (r.error || "Re-enable failed"), r.ok ? "success" : "error");
  tick();
};
document.getElementById("gozero").onclick = async () => {
  if (last.estopped) { toast("Motors are disabled — re-enable first", "warn"); return; }
  if (atZero()) { toast("Already at zero", "info"); return; }
  toast("Returning to zero…", "info");
  const r = await post("/return_zero");
  if (!r.ok) toast(r.error || "Return failed", "error");
  else if (!r.estopped) toast("Back at zero", "success");
  tick();
};
document.getElementById("zero").onclick = async () => {
  if (!confirm("Set current position as zero (new home reference)?")) return;
  const r = await post("/zero");
  toast(r.ok ? "Zero set at current position" : (r.error || "Failed to set zero"),
        r.ok ? "success" : "error");
  tick();
};
document.getElementById("home").onclick = async () => {
  if (last.estopped) { toast("Motors are disabled — re-enable first", "warn"); return; }
  if (!confirm("Home all axes? X and Y seek their switches; Z returns to zero.")) return;
  toast("Homing axes…", "info", 5000);
  const r = await post("/home");
  if (!r.ok) toast(r.error || "Homing failed", "error");
  else if (!r.estopped) toast("Homing complete", "success");
  tick();
};
document.getElementById("desktop").onclick = () => { toast("Exiting to desktop…", "info"); post("/kiosk-exit"); };
document.getElementById("reboot").onclick = () => { if (confirm("Reboot the Pi?")) { toast("Rebooting…", "warn", 8000); post("/reboot"); } };
document.getElementById("poweroff").onclick = () => { if (confirm("Power OFF the Pi?")) { toast("Powering off…", "warn", 8000); post("/poweroff"); } };
tick(); setInterval(tick, 1500);
</script></body></html>"""
