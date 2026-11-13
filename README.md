# Energy-Aware Cruise Control on Uncertain Hilly Roads

A **complete, runnable, educational Python simulation** of longitudinal passenger-car cruise control. It compares a conventional PI controller against a PI controller with resistance feedforward and *limited, imperfect* road-grade preview.

**Engineering question:** When the car approaches hills and descents, can a controller that knows an imperfect road-grade estimate reduce tracking error and/or energy use without increasing acceleration and jerk? What happens when its preview is wrong?

This is **not** a claim of an unprecedented algorithm, an optimal controller, production-grade ADAS, or validated real-car performance. The contribution is a transparent and reproducible *problem definition* and comparison under a combined set of constraints. The controllers are simple, and one may perform worse than the other in some scenarios.

## What is modeled

- 1D longitudinal dynamics, position-based 6% uphill and downhill sections
- Quadratic aerodynamic drag, rolling resistance, gravity
- Force and speed-dependent drive-power limits
- Friction braking and simplified regenerative braking with force/power caps
- PI control with back-calculation anti-windup
- Alternative controller: PI + current resistance feedforward + imperfect grade preview
- Vehicle mass, headwind, and grade-preview bias sensitivity
- Speed error, overspeed, force saturation, acceleration, jerk, drive energy, recovered energy, friction-braking losses
- CSV/JSON output and optional matplotlib comparison plot
- Automated physics and simulation tests

## Road profile

| Distance | Grade |
|---|---|
| 0–600 m | level |
| 600–1400 m | +6% uphill |
| 1400–1900 m | level |
| 1900–2700 m | −6% downhill |
| 2700+ m | level |

The profile uses **distance, not time**. Grade is rise/run (0.06), converted to angle with `atan` for force calculations.

## Dynamics and energy

For forward motion:

```text
m dv/dt = F_applied - F_aero - F_roll - F_grade
F_aero  = 0.5 rho Cd A (v + headwind) |v + headwind|
F_roll  = Crr m g cos(atan(grade))
F_grade = m g sin(atan(grade))
F_drive,max(v) = min(F_drive,cap, P_drive,cap / max(v, 0.1))
```

Positive wheel force is traction, negative is braking. Regeneration is limited by force and power; remaining braking is friction. Electrical energy is approximated from positive wheel power divided by drivetrain efficiency, and recovered wheel power multiplied by regeneration efficiency. This simplified model excludes battery SOC-dependent limits, engine maps, HVAC, and tire-road friction.

**Important modeling boundaries:** No reverse gear, no detailed tire slip, no sensor dynamics, no engine or brake actuator lag, no terrain smoothness model, no formal safety guarantees. Grade transitions are discontinuous and can produce artificial jerk spikes. Energy figures are *comparative estimates*, not validated consumption predictions. Battery state is bounded for display but the simulator does not reduce propulsion when the battery is depleted. Initial battery is 80% SOC.

## Install and run

Python 3.10+ is required. The simulation and tests use only the standard library.

```bash
python -m pip install -e .
python -m unittest discover -s tests -v
python -m energy_cruise.cli --controller both --output results
```

On Windows use `py` in place of `python` if needed. To plot results:

```bash
python -m pip install -e '.[plot]'
python examples/plot_results.py
```

Try uncertain-preview and heavier-car experiments:

```bash
python -m energy_cruise.cli --controller both --preview-bias 0.03 --output results/biased
python -m energy_cruise.cli --controller both --mass-scale 1.3 --headwind 8 --output results/heavy_windy
python -m energy_cruise.cli --controller both --initial-kmh 100 --target-kmh 60 --output results/braking
```

The script writes `pi.csv`, `preview.csv`, and matching JSON metric summaries. The comparison PNG is optional.

## Parameters

| Parameter | Default |
|---|---:|
| Mass | 1500 kg |
| Reference / initial speed | 100 km/h |
| Drag coefficient / area | 0.30 / 2.2 m² |
| Air density / rolling coefficient | 1.225 kg/m³ / 0.012 |
| Maximum traction / braking force | 4500 / 8000 N |
| Traction power cap | 80 kW |
| Regen power / force cap | 25 kW / 1600 N |
| Drive / regen efficiency | 0.90 / 0.65 |
| PI Kp / Ki / anti-windup | 850 / 55 / 0.5 |
| Preview distance / gain | 100 m / 0.35 |
| Simulation timestep | 0.05 s |

Gains are illustrative and **not optimized**. `preview_gain` blends only the grade-dependent feedforward component. The preview controller sees current grade and a single estimate at a fixed future distance; it does not see future vehicle speed or the true future trajectory.

## Interpreting results

Do not claim that preview control is better unless the measured results show that for the stated scenario and metric. The main trade-offs are speed tracking, energy consumption, braking losses, and ride comfort. A lower speed error does not imply lower energy use, and a lower battery-energy estimate may reflect poor speed tracking.

`rms_jerk_m_s3` includes grade-step discontinuities and is timestep-sensitive. The CSV reports state and actuation at the **start** of each integration interval; the simulator advances the vehicle with a fixed-step forward-Euler speed update and trapezoidal position update. The final position and final speed at the exact end time are not included as a CSV row. The `distance_km` summary uses the last recorded sample position and therefore slightly underreports the final distance. Friction-braking energy uses a fixed dt and is approximate if the final interval is partial.

## Project layout

```text
src/energy_cruise/
  model.py         # vehicle, environment, road and actuators
  controllers.py   # PI and limited-preview PI
  simulation.py    # deterministic fixed-step simulation
  metrics.py       # reproducible comparisons
  cli.py           # scenario CLI and CSV/JSON export
examples/plot_results.py
 tests/             # unit tests
```

## Further research directions

1. Replace the simplistic grade-step profile with measured or smoothly interpolated GPS elevation data and an error model.
2. Calibrate against real vehicle telemetry and quantify uncertainty intervals.
3. Optimize speed trajectories under hard time, comfort, traction, and energy constraints (e.g., MPC) and compare against the baseline on equal trip-time/distance criteria.
4. Include a physically consistent SOC-dependent regeneration and traction limit.

## Originality and prior work

Cruise control, grade feedforward, anti-windup, energy-aware control, and predictive braking all have extensive prior art. This repository does **not** claim that these methods are new. Its specific reproducible experiment combines distance-indexed grade disturbances, biased preview, power-limited traction, split regenerative/friction braking, and explicit multi-objective metrics in a compact, inspectable standard-library codebase.

Useful references:
- [Python Control Systems Library cruise-control example](https://python-control.readthedocs.io/en/latest/examples/cruise.html)
- [PyCruiseControl](https://github.com/komrad36/PyCruiseControl)
- [Vehicle speed control with anti-windup](https://github.com/volkanakyazici/vehicle-speed-control)
- [CommonRoad safe-acc](https://github.com/CommonRoad/safe-acc)

## License

MIT. See `LICENSE`.
