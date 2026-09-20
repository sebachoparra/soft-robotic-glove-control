> **Frozen golden reference.** This directory (`reference/rp2040_validated/`, tagged `rp2040-golden-v1`) is preserved as-is and must not be modified. It is the validated, behaviorally-frozen standalone RP2040 firmware that the ROS 2 migration (`ros2_ws/`, `pico_firmware/`) is required to reproduce to numerical tolerance. See the project's governance policy for the full policy.

> Actualización: leer primero GAIN_SCHEDULING.md. Las tablas y notas de scheduling histórico quedan sustituidas por el modelo continuo.

# UKF4 selectable feedback test: PI + ADRC

Firmware de prueba para comparar el mismo sistema usando FLEX o el estado `s_hat` del UKF4 como realimentación externa.

## Modos

- `CTRL,PI` — PI externo con feedback FLEX.
- `CTRL,PI_UKF` — mismo PI externo con feedback `UKF4 s_hat`.
- `CTRL,ADRC` — ADRC/LESO original con feedback FLEX y el feedforward identificado original.
- `CTRL,ADRC_UKF` — mismo ADRC/LESO y **mismo feedforward** pero usando `UKF4 s_hat` como medición de posición.
- `CTRL,PRESSURE` — lazo de posición abierto, lazo de presión cerrado.

El lazo PI interno de presión, FILL/HOLD/VENT, rate limiter y protecciones se mantienen comunes.

## ADRC_UKF

En `CTRL,ADRC_UKF`:

- el LESO recibe `s_hat` en lugar de `q_FLEX`;
- `ladrc_update()` se mantiene igual;
- `select_adrc_model()` y `compute_adrc_feedforward()` se mantienen iguales;
- los parámetros `K`, `tau`, `b0`, `step_ff_kpa` y `ff_gamma` siguen siendo los del ADRC anterior;
- FLEX sigue entrando como medición al UKF4 y sigue registrándose, pero no es el feedback directo del ADRC.

Si UKF4 deja de ser válido en `PI_UKF` o `ADRC_UKF`, el firmware entra en una retención segura de `P_ref` alrededor de la presión medida en vez de seguir llenando.

## Primera secuencia recomendada

`20 -> 40 -> 60 -> 80 -> 90 -> 80 -> 60 -> 40 -> 20 %`

Las regiones ADRC identificadas exactas llegan hasta 60->80 y 80->60. Por eso el feedforward identificado se aplica hasta 80%. Los tramos 80->90 y 90->80 se ejecutan con el fallback actual (sin feedforward identificado) para no inventar un modelo no identificado.

## Capturador

Use `pico_ukf4_pi_adrc_feedback_test.py` y cambie:

```python
CONTROLLER = "ADRC_UKF"
```

Para comparar, repetir luego con `ADRC`, `PI_UKF` y/o `PI` sin cambiar el resto de la secuencia.

## Verificación

`STATUS` debe reportar `#UKF4_STATUS` y, en el modo nuevo:

`CTRL=ADRC_UKF` y `QFB_SRC=UKF4_S`.

El CSV contiene `q_feedback_pct` y `q_feedback_source`, además de los estados UKF4.

Host syntax check: PASS con `gcc -std=c11 -Wall -Wextra -Wformat=2 -fsyntax-only` usando stubs Pico. Compilar con el Pico SDK real antes de flashear.
