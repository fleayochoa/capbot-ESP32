# Diagramas de desempeño del PID

`plot_run.py` corre **en el PC** (host): lee los CSV que generan los scripts
de tuning y produce diagramas PNG con métricas de la respuesta (rise time,
overshoot, settling, error estacionario) o la caracterización PWM↔velocidad.

Los scripts que generan esos CSV (`ramp_test.py`, `step_test.py`) corren en
la **Jetson** — hablan directo por serial con el ESP32 — y viven en
[`capbot-jetson/scripts/pid_tuning/`](../../capbot-jetson/scripts/pid_tuning/),
no aquí, porque ese es el repo que realmente está desplegado en la Jetson
(`jetson-service`). Ver el README de esa carpeta para el flujo completo.

Este repo (`capbot-ESP32`) sigue siendo el dueño de los parámetros que se
tunean: `DEFAULT_CTRL_CFG`, `PID_PARAM` ids y `Cfg::WHEEL_CPR` en
`src/main.cpp` / `include/Config.h`.

## Uso

```bash
# necesita matplotlib: pip install matplotlib
python tools/plot_run.py step_banco.csv          # respuesta al escalón + métricas
python tools/plot_run.py ramp_banco.csv --ramp   # vel vs PWM + ajuste lineal
python tools/plot_run.py step_banco.csv --out figura.png
```

## Flujo completo (resumen)

1. En la Jetson: `ramp_test.py` (Fase 2, caracterización) y `step_test.py`
   (Fase 3, escalón de 5 rad/s) — ver
   `capbot-jetson/scripts/pid_tuning/README.md`.
2. Copiar los CSV resultantes al PC (`scp`).
3. `plot_run.py <csv>` aquí, en el PC.
4. Fijar los valores ganadores en `DEFAULT_CTRL_CFG` (este repo) y reflashear.
