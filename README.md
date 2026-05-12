# Rebuilt 2026

RobotPy code for FRC Team 4611's robot in the custom **Rebuilt** game.

## Game Context

This codebase is built around the Rebuilt field model defined in the robot code:

- The primary scoring objective is the **hub**, represented in code as `GamePositions.HUB`.
- The field includes **trenches**, trench-assist regions, and a **slow zone** via `SoftWalls`.
- Match flow includes alliance-dependent **shift timing**, handled by `ShiftScheduler`.
- The robot supports three main scoring modes:
  - close hub shots
  - mid-field passes
  - full-field passes

Most game-specific geometry and timing rules live in `utilities/rebuilt_helpers.py`.

## Robot Architecture

The robot is organized as a MagicBot/RobotPy project with subsystem-style components.

### Core entrypoint

- `robot.py`: top-level robot lifecycle, controller bindings, energy profile switching, driver assist logic, and vision setup

### Major components

- `components/drivetrain.py`
  - 4-module swerve drivetrain
  - odometry and pose estimation
  - snap-to-angle, line-up, lock, snake, and driver-assist states
  - trajectory following for Choreo autonomous paths

- `components/shooter.py`
  - 3-motor flywheel shooter
  - feeder control
  - hood angle control with CANcoder feedback
  - interpolation-driven shot settings and pass presets

- `components/intake.py`
  - intake roller velocity control
  - pivoted intake with profiled PID + feedforward
  - stash, deploy, intake, eject, and agitation states

- `components/serializer.py`
  - internal note transport between intake and shooter

- `components/superstructure.py`
  - high-level coordination layer
  - manages intake, stash, shoot, pass, unjam, and eject behaviors

- `components/vision.py`
  - Limelight-based pose updates
  - camera disable selection and fused pose logging
  - integrates vision measurements into drivetrain pose estimation

## Autonomous

Autonomous routines are Choreo-driven.

- `autonomous/base_autonomous.py`
  - shared autonomous state machine
  - handles path following, stationary actions, bump moves, line-up moves, and snap-to-rotation actions

- `autonomous/autons.py`
  - match-ready auton definitions built from `PathProperty` sequences

- `deploy/choreo/`
  - deployed Choreo trajectories consumed at runtime

## Utilities

- `utilities/configs.py`: typed config objects for drivetrain, shooter, intake, serializer, vision, and climber
- `utilities/helpers.py`: shared math, drive signal types, tolerances, and path metadata
- `utilities/rebuilt_helpers.py`: Rebuilt field constants, soft walls, shift timing, interpolation, and pass-angle helpers
- `utilities/energy_profiles.py`: current-limit profiles for normal shooting, passing, overclock, and brownout handling
- `utilities/IO.py` and `utilities/IO_helpers.py`: logging/publishing helpers for telemetry and dashboards
- `utilities/states.py`: enum-based subsystem states

## Project Layout

```text
.
├── autonomous/
├── components/
├── deploy/
│   └── choreo/
├── tests/
├── utilities/
├── physics.py
├── pyproject.toml
└── robot.py
```

## Development

This project targets:

- `robotpy_version = 2026.2.2`
- `phoenix6`
- `sleipnirgroup-choreolib==2026.0.0`

Typical RobotPy workflow:

```bash
python3 -m pip install robotpy
python3 -m robotpy sync
python3 robot.py sim
```

Deploy to the roboRIO with:

```bash
python3 -m robotpy deploy
```

## Notes

- The repo includes `physics.py` for simulation support.
- Tests live in `tests/`, though your local environment may need extra Python packages installed before they can run.
- Dashboard and telemetry support are heavily used throughout the codebase for tuning and match diagnostics.
