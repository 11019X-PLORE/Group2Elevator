# Profile generator

Generates the elevator's motion profiles: minimum-time, jerk-limited trajectories from a
CasADi/IPOPT direct-collocation solve. Refactored in from the team's standalone
`profileGeneratorV2.0` folder — same optimization core, but config-driven, batch, and with
metadata written into every output file.

## What it solves

Each joint is a triple integrator (position/velocity/acceleration states, jerk as the control
input) under box constraints. Total time `T` is a decision variable and the objective is
minimum time plus a small velocity regularization for solver conditioning (the position
regularization that used to bias the old generator's solutions toward "stay low" now defaults
to zero). Boundary conditions: at rest with zero acceleration at both ends. Waypoints may be
passed through in between, spread evenly over the grid.

Units are whatever the config says — no conversion happens here. `configs/elevator-demo.json`
works in **travel-fraction** (0 = the calibrated zero, 1 = full extension), so profiles load
on the robot with no scaling and can never describe a position outside the calibrated travel.

## Install and run

```sh
cd tools/profile-generator
python3 -m venv .venv          # casadi 3.6.7 needs Python 3.9–3.12
source .venv/bin/activate
pip install -r requirements.txt

python generate_profiles.py --config configs/elevator-demo.json
```

Outputs land in `src/main/deploy/profiles/` by default (that directory is deployed to the
roboRIO and read by `MotionProfile.fromJson`). Options:

- `--profile NAME` — generate only the named profile (repeatable)
- `--out DIR` — write somewhere else
- `--plot NAME` — pop an inspection plot of position/velocity/acceleration

## Workflow

1. Characterize the rig (see the dynamics-characterization section of `docs/CALIBRATION.md`):
   measure the real velocity/acceleration envelope in travel-fraction units.
2. Fill those numbers into the config's joint limits, and replace the PLACEHOLDER note.
3. Regenerate, commit both the config and the produced JSON — the profile files carry the
   limits and note in their metadata, so a trajectory can be audited from the file alone.
4. `./gradlew test` includes a test that re-validates every file in `src/main/deploy/profiles`
   (monotonic time, starts at t = 0, positions inside [0, 1]), so a bad file fails CI instead
   of a match.

## Config schema

```jsonc
{
  "solver":        { "intervals": 200, "min_time_seconds": 0.1, "max_time_seconds": 10.0 },
  "regularization": { "velocity": 0.01, "acceleration": 0.0, "position": 0.0 },
  "joints": [                                 // 1 joint = elevator-only; 2+ for a coupled rig
    { "name": "elevator", "units": "travel-fraction", "min": 0.0, "max": 1.0,
      "max_velocity": 0.5, "max_acceleration": 1.0, "max_jerk": 25.0 }
  ],
  "profiles": [                              // one output file per entry
    { "name": "zeroToWork", "waypoints": [[0.0], [0.75]] }
  ],
  "notes": "carried into every generated file's metadata"
}
```

Each waypoint lists one position per joint, in joint order. The first and last are the start
and goal; intermediate ones are passed through. Multi-joint files store `x`/`v`/`a` as arrays
(the robot-side loader currently consumes the single-joint scalar form only).
