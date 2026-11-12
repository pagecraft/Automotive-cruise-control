# Automotive Cruise Control — Engineering Starter

A deliberately incomplete Python project for a three-turn coding-agent evaluation.

## Real-world scenario
A 1,500 kg passenger car should hold 100 km/h on a highway despite aerodynamic drag, rolling resistance, and changes in road grade.

- 0–20 s: level
- 20–50 s: 6% uphill
- 50–70 s: level
- 70–100 s: 6% downhill

The grade is rise/run, not degrees. Road boundaries are left-inclusive.

## Vehicle parameters
- Mass: 1500 kg
- Drag coefficient: 0.30
- Frontal area: 2.2 m²
- Air density: 1.225 kg/m³
- Rolling resistance coefficient: 0.012
- Max driving force: 4500 N
- Max braking force: 8000 N

## Dynamics
m dv/dt = F_drive - F_brake - F_aero - F_roll - F_grade

For forward motion:
F_aero = 0.5 rho Cd A v²
F_roll = Crr m g cos(arctan(grade))
F_grade = m g sin(arctan(grade))

## Running the starter
Python 3.10+; no external dependencies required for the starter.

    py -m unittest discover -s tests -v
    py examples/highway_drive.py

On macOS/Linux use `python3` instead of `py`.

## Agent evaluation roadmap
Turn 1: implement physical vehicle dynamics, PI control, simulation, numerical verification, and performance metrics.
Turn 2: evaluate and correct saturation/integral-windup behavior.
Turn 3: validate robustness under changing mass, road gradient, and disturbances.

The starter tests only cover the road schedule and default configuration. They do not validate the unfinished controller or vehicle dynamics.
