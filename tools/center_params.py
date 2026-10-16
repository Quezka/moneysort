"""Settings shared by center_coin.py and center_settings.py (no hardware, no HTTP).

Each setting: key -> (default, kind, min, max, Italian description). center_coin.py reads the
saved values from center_settings.json at start and overrides its module constants; the drop spot
lives in the same file under "drop" ([x, y, z], base frame, mm).
"""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "center_settings.json")

# key: (default, kind, min, max, description)
PARAMS = {
    "PPS":                (6000, int, 1000, 12000, "velocita' massima dei movimenti (impulsi/s); sopra 8000 aumenta il rischio di blocchi"),
    "SETTLE_S":           (0.25, float, 0.0, 2.0, "attesa dopo ogni mossa prima di guardare (s): camera lenta = piu' alta"),
    "GAIN":               (1.0, float, 0.2, 1.2, "quanta parte dell'errore corregge a ogni passo (1.0 = tutto)"),
    "MAX_STEP_MM":        (60.0, float, 5.0, 150.0, "passo massimo di correzione (mm)"),
    "TOL_PX":             (10, int, 2, 60, "moneta 'centrata' entro questi pixel dal bersaglio"),
    "FINISH_PX":          (30, int, 0, 100, "entro questi pixel smette di centrare e corregge durante la discesa (0 = mai)"),
    "TRACK_GATE_PX":      (50, int, 10, 150, "raggio (px) in cui accetta una moneta come 'la stessa' dopo una mossa"),
    "PROBE_MM":           (12.0, float, 4.0, 40.0, "ampiezza delle due mosse di prova per tarare le mosse (mm)"),
    "APPROACH_MARGIN_MM": (15.0, float, 0.0, 60.0, "distanza tenuta sopra la moneta in discesa (mm), se non c'e' una calibrazione"),
    "MAX_DROP_MM":        (50.0, float, 0.0, 150.0, "quanto puo' abbassarsi per raggiungere una moneta lontana, durante la centratura (mm)"),
    "PUMP_FITTED":        (False, bool, None, None, "pompa montata: dopo una moneta prosegue da solo fino a svuotare la pila"),
}


def load():
    try:
        with open(PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save(data):
    with open(PATH, "w") as f:
        json.dump(data, f, indent=2)


def values(data=None):
    """Current value of every setting (saved, else default)."""
    data = load() if data is None else data
    saved = data.get("params", {})
    return {k: saved.get(k, spec[0]) for k, spec in PARAMS.items()}


def parse(key, text):
    """Text -> typed, range-checked value. Raises ValueError with a readable message."""
    default, kind, lo, hi, _ = PARAMS[key]
    t = text.strip().lower()
    if kind is bool:
        if t in ("1", "true", "si", "s", "on", "yes", "y"):
            return True
        if t in ("0", "false", "no", "n", "off"):
            return False
        raise ValueError("rispondi si' o no")
    try:
        v = kind(float(t)) if kind is int else float(t)
    except ValueError:
        raise ValueError("serve un numero")
    if v < lo or v > hi:
        raise ValueError(f"tra {lo} e {hi}")
    return v
