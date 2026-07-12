"""Diagramas de desempeño del PID a partir de los CSV de step_test/ramp_test.

Corre en el PC (necesita matplotlib). Uso:

    python plot_run.py step_5rads.csv              # step: respuesta + métricas
    python plot_run.py ramp_banco.csv --ramp       # rampa: vel vs PWM + ajuste
    python plot_run.py step.csv --out figura.png

Para un step genera 3 paneles con eje de tiempo común:
  1. Rueda izquierda : setpoint vs velocidad medida (rad/s)
  2. Rueda derecha   : setpoint vs velocidad medida (rad/s)
  3. Esfuerzo PWM de ambas ruedas (counts)
y calcula por rueda: rise time (10-90%), overshoot, settling time (±5%) y
error estacionario, impresos y anotados en la figura.
"""
import argparse
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- Paleta validada (referencia dataviz, modo claro) ----
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
LEFT_HUE = "#2a78d6"    # rueda izquierda (azul, slot 1)
RIGHT_HUE = "#1baf7a"   # rueda derecha (aqua, slot 2)

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "text.color": INK,
    "axes.labelcolor": INK_2,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "axes.edgecolor": AXIS,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.titlecolor": INK,
})


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        first = f.readline()
        comment = first[1:].strip() if first.startswith("#") else None
        if not first.startswith("#"):
            f.seek(0)
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "t": float(r["t"]),
                    "sp_left": float(r["sp_left"]), "sp_right": float(r["sp_right"]),
                    "vel_left": float(r["vel_left"]), "vel_right": float(r["vel_right"]),
                    "pwm_left": float(r["pwm_left"]), "pwm_right": float(r["pwm_right"]),
                    "cmd_a": float(r["cmd_a"]), "cmd_b": float(r["cmd_b"]),
                })
            except (KeyError, ValueError):
                continue
    return rows, comment


# ------------------------------------------------------------
# Métricas de step
# ------------------------------------------------------------
def step_metrics(t, sp, vel):
    """Métricas clásicas de respuesta al escalón. Retorna dict o None."""
    # Ventana del step: sp distinto de 0 sostenido.
    idx = [i for i, s in enumerate(sp) if abs(s) > 0.01]
    if len(idx) < 10:
        return None
    i0, i1 = idx[0], idx[-1]
    target = sp[i1]  # valor final del setpoint durante el step
    if abs(target) < 0.01:
        return None
    sign = 1.0 if target > 0 else -1.0
    t0 = t[i0]
    # Normalizar al signo del step para que las métricas valgan en reversa.
    v = [sign * x for x in vel[i0:i1 + 1]]
    ts = t[i0:i1 + 1]
    tgt = sign * target

    def first_crossing(level):
        for k, x in enumerate(v):
            if x >= level:
                return ts[k]
        return None

    t10 = first_crossing(0.1 * tgt)
    t90 = first_crossing(0.9 * tgt)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

    peak = max(v)
    overshoot = max(0.0, (peak - tgt) / tgt * 100.0)

    # Settling ±5%: último instante fuera de banda.
    settle = None
    band = 0.05 * abs(tgt)
    outside = [ts[k] for k, x in enumerate(v) if abs(x - tgt) > band]
    if not outside:
        settle = 0.0
    elif outside[-1] < ts[-1]:
        settle = outside[-1] - t0

    # Error estacionario: promedio del último 25% de la ventana.
    n_tail = max(1, len(v) // 4)
    ss = tgt - sum(v[-n_tail:]) / n_tail

    return {"target": target, "t0": t0, "rise": rise, "overshoot": overshoot,
            "settle": settle, "ss_error": sign * ss}


def fmt_metrics(name, m):
    if m is None:
        return "{}: sin step detectado".format(name)
    rise = "{:.2f} s".format(m["rise"]) if m["rise"] is not None else "n/a"
    settle = "{:.2f} s".format(m["settle"]) if m["settle"] is not None else "no asienta"
    return ("{}: rise(10-90%)={}  overshoot={:.1f}%  settling(±5%)={}  "
            "error ss={:+.2f} rad/s".format(name, rise, m["overshoot"], settle, m["ss_error"]))


# ------------------------------------------------------------
# Figuras
# ------------------------------------------------------------
def plot_step(rows, out, comment):
    t = [r["t"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9.5), sharex=True)
    fig.subplots_adjust(hspace=0.5, top=0.92, bottom=0.11)

    results = []
    for ax, wheel, hue in ((axes[0], "left", LEFT_HUE), (axes[1], "right", RIGHT_HUE)):
        sp = [r["sp_" + wheel] for r in rows]
        vel = [r["vel_" + wheel] for r in rows]
        m = step_metrics(t, sp, vel)
        results.append((wheel, m))

        ax.plot(t, sp, color=INK_2, linewidth=1.6, linestyle="--", label="Setpoint")
        ax.plot(t, vel, color=hue, linewidth=2.0, label="Medida")
        if m and m["target"]:
            band = 0.05 * abs(m["target"])
            ax.axhspan(m["target"] - band, m["target"] + band, color=GRID, alpha=0.5, zorder=0)
        title = "Rueda izquierda" if wheel == "left" else "Rueda derecha"
        ax.set_title(title, loc="left", pad=22)
        ax.set_ylabel("Velocidad (rad/s)")
        ax.legend(loc="lower right", fontsize=9)
        if m:
            txt = fmt_metrics("", m).lstrip(": ")
            ax.text(0.0, 1.03, txt, transform=ax.transAxes, fontsize=8.5,
                    color=INK_2, va="bottom")

    ax = axes[2]
    ax.plot(t, [r["pwm_left"] for r in rows], color=LEFT_HUE, linewidth=2.0, label="Izquierda")
    ax.plot(t, [r["pwm_right"] for r in rows], color=RIGHT_HUE, linewidth=2.0, label="Derecha")
    ax.set_title("Esfuerzo de motor", loc="left")
    ax.set_ylabel("PWM (counts)")
    ax.set_xlabel("Tiempo (s)")
    ax.legend(loc="upper left", fontsize=9)
    # Etiqueta directa al final de cada línea (regla de relieve para el aqua).
    if rows:
        ax.annotate("izq", (t[-1], rows[-1]["pwm_left"]), color=LEFT_HUE,
                    fontsize=8.5, xytext=(4, 0), textcoords="offset points", va="center")
        ax.annotate("der", (t[-1], rows[-1]["pwm_right"]), color=RIGHT_HUE,
                    fontsize=8.5, xytext=(4, -10), textcoords="offset points", va="center")

    fig.suptitle("Respuesta al escalón — PID de velocidad por rueda", fontsize=13,
                 fontweight="bold", x=0.02, ha="left")
    if comment:
        fig.text(0.02, 0.02, comment, fontsize=7.5, color=MUTED)
    fig.savefig(out, dpi=150)
    print("Figura guardada: {}".format(out))
    for wheel, m in results:
        print(fmt_metrics("Rueda {:5s}".format("izq." if wheel == "left" else "der."), m))


def plot_ramp(rows, out, comment):
    """Rampa: vel de régimen vs PWM comandado (cmd_a) + ajuste lineal."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)
    fig.subplots_adjust(top=0.86, bottom=0.14, wspace=0.08)

    for ax, wheel, hue in ((axes[0], "left", LEFT_HUE), (axes[1], "right", RIGHT_HUE)):
        # Agrupar por nivel de PWM comandado; régimen = mediana del nivel.
        by_level = {}
        for r in rows:
            cmd = r["cmd_a"]
            if cmd != 0:
                by_level.setdefault(cmd, []).append(r["vel_" + wheel])
        pts = []
        for cmd, vels in sorted(by_level.items()):
            # vels está en orden cronológico: régimen = mediana del último 40%
            # del hold (descarta el transitorio del cambio de nivel).
            tail = sorted(vels[int(len(vels) * 0.6):]) or vels
            pts.append((tail[len(tail) // 2], cmd))
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=28, color=hue,
                   edgecolors=SURFACE, linewidths=1.2, zorder=3, label="Niveles medidos")

        moving = [(abs(v), abs(p)) for v, p in pts if abs(v) > 0.3]
        if len(moving) >= 2:
            n = len(moving)
            sx = sum(p[0] for p in moving); sy = sum(p[1] for p in moving)
            sxx = sum(p[0] ** 2 for p in moving); sxy = sum(p[0] * p[1] for p in moving)
            den = n * sxx - sx * sx
            if abs(den) > 1e-9:
                b = (n * sxy - sx * sy) / den
                a = (sy - b * sx) / n
                vmax = max(p[0] for p in moving)
                ax.plot([0, vmax], [a, a + b * vmax], color=INK_2, linestyle="--",
                        linewidth=1.6, label="pwm = {:.0f} + {:.0f}·vel".format(a, b))
        title = "Rueda izquierda" if wheel == "left" else "Rueda derecha"
        ax.set_title(title, loc="left")
        ax.set_xlabel("Velocidad de régimen (rad/s)")
        ax.legend(loc="lower right", fontsize=9)
    axes[0].set_ylabel("PWM comandado (counts)")

    fig.suptitle("Caracterización PWM → velocidad (feedforward kStatic + kV·vel)",
                 fontsize=13, fontweight="bold", x=0.02, ha="left")
    if comment:
        fig.text(0.02, 0.02, comment, fontsize=7.5, color=MUTED)
    fig.savefig(out, dpi=150)
    print("Figura guardada: {}".format(out))


def main():
    ap = argparse.ArgumentParser(description="Diagramas de desempeño del PID")
    ap.add_argument("csv", help="CSV de step_test.py o ramp_test.py")
    ap.add_argument("--ramp", action="store_true", help="el CSV es de ramp_test.py")
    ap.add_argument("--out", default=None, help="PNG de salida (default: <csv>.png)")
    args = ap.parse_args()

    rows, comment = load_csv(args.csv)
    if not rows:
        sys.exit("CSV vacío o ilegible: {}".format(args.csv))
    out = args.out or (args.csv.rsplit(".", 1)[0] + ".png")

    if args.ramp:
        plot_ramp(rows, out, comment)
    else:
        plot_step(rows, out, comment)


if __name__ == "__main__":
    main()
