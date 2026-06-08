# Simulation — Omnidirectional Robot Obstacle Avoidance

## Purpose

- Minimal simulation of an omnidirectional robot following a reference trajectory while avoiding a static obstacle using artificial potential fields.

Requirements

- Python 3.8 or later
- numpy
- matplotlib

## Installation

- Create a virtual environment (recommended) and install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install numpy matplotlib
```

## Running

- Run the simulation and animation:

```bash
python main.py
```

## Configuration

- Edit the `SimulationConfig` dataclass in [main.py](main.py) to change `dt`, `t_end`, `k_att`, `k_rep`, or `rho_0`.
- The robot initial position and obstacle position are set in `run_simulation()` in [main.py](main.py).

What the script does

- Generates a time-varying reference trajectory.
- Uses a kinematic controller (attractive + repulsive potentials) to compute velocity commands.
- Integrates the robot position and displays an animated plot showing:
  - reference trajectory
  - obstacle position
  - robot trajectory and current position

## Notes

- The repulsive force is active only within the `rho_0` radius around the obstacle.
- Increase `k_rep` or `rho_0` to strengthen or enlarge the avoidance behavior.
