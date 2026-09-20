# data/

## data/pressure_tests/

This directory holds raw CSV logs captured during pressure/position sweep
experiments (e.g. `pressure_sweep_20_150_*.csv`, `pi_ukf_sweep_20_80_*.csv`,
`adrc_ukf_sweep_20_80_*.csv`, `pressure_sweep_full_sensing_*.csv`).

These CSVs are **not tracked in git** (`data/pressure_tests/*.csv` is in
`.gitignore`) because they are bulky, raw experimental output, not source
code or documentation, and are cheap to regenerate on demand from the
device.

### Regenerating the data

Run the corresponding capture script from `tools/pressure/` or
`tools/position/` against a live Pico + ROS 2 stack:

- `tools/pressure/run_calibration.sh` — zero-calibration bring-up sequence.
- `tools/pressure/pressure_sweep_20_120.py`, `tools/pressure/pressure_sweep_20_150.py` —
  internal pressure-PI sweep captures.
- `tools/pressure/full_sensing_logger.py` — full-sensing CSV logger.
- `tools/position/pi_ukf_sweep_20_80.py`, `tools/position/adrc_ukf_sweep_20_80.py` —
  position-controller (PI/ADRC, UKF feedback) sweep captures.

Each script writes a timestamped CSV into `data/pressure_tests/` (or the
directory it is run from). See `docs/calibration.md` and
`docs/pressure_control.md` for the calibration/experiment workflow these
scripts belong to.

No hardware actuation should be performed without following the safety
notes in `docs/PRESSURE_BRINGUP.md` and `docs/troubleshooting.md`.
