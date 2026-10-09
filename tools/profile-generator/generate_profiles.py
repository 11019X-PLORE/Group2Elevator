#!/usr/bin/env python3
"""Minimum-time, jerk-limited point-to-point motion profiles.

Refactored from profileGeneratorV2.0 (CasADi + IPOPT). The optimization core is
unchanged in spirit: one triple-integrator per joint with jerk as the control
input, box constraints on position/velocity/acceleration/jerk, and total time as
a decision variable. What used to be hand-edited source — waypoints, limits,
output names — now lives in a JSON config, profiles are generated in one batch,
and every output file carries its own metadata so a trajectory can be audited
without the config that produced it.

Units: whatever the config says. The solver does no conversion; it optimizes in
the joint units it is given. ElevatorDemo works in travel-fraction (0 = the
calibrated zero, 1 = full extension) so profiles load on the robot with no
scaling — see configs/elevator-demo.json.

Usage:
    python3 generate_profiles.py --config configs/elevator-demo.json
    python3 generate_profiles.py --config configs/elevator-demo.json \
        --profile zeroToWork --plot zeroToWork
"""

import argparse
import datetime
import json
import sys
from pathlib import Path

import casadi as ca

GENERATOR_VERSION = "profile-generator 2.1"

# Solver results below this magnitude are numerical noise (the original generator
# wrote values like -1.7e-42 for "at rest at zero"); they are written as exact 0.
NOISE_FLOOR = 1e-9


class Joint:
    """One degree of freedom and the limits the solver must respect."""

    def __init__(self, spec):
        self.name = spec["name"]
        self.units = spec.get("units", "unspecified")
        self.min = float(spec["min"])
        self.max = float(spec["max"])
        self.max_velocity = float(spec["max_velocity"])
        self.max_acceleration = float(spec["max_acceleration"])
        self.max_jerk = float(spec["max_jerk"])
        if not self.min < self.max:
            raise ValueError(f"joint {self.name}: min must be below max")
        for label, value in (
            ("max_velocity", self.max_velocity),
            ("max_acceleration", self.max_acceleration),
            ("max_jerk", self.max_jerk),
        ):
            if value <= 0.0:
                raise ValueError(f"joint {self.name}: {label} must be positive")

    def limits(self):
        return {
            "min": self.min,
            "max": self.max,
            "max_velocity": self.max_velocity,
            "max_acceleration": self.max_acceleration,
            "max_jerk": self.max_jerk,
        }


def solve_profile(profile_cfg, joints, solver_cfg, regularization):
    """Solves one profile; returns (optimal time, list of sample dicts)."""
    n_joints = len(joints)
    intervals = int(solver_cfg.get("intervals", 200))
    waypoints = [[float(value) for value in waypoint] for waypoint in profile_cfg["waypoints"]]
    if len(waypoints) < 2:
        raise ValueError(f"profile {profile_cfg['name']}: needs at least a start and an end")
    for waypoint in waypoints:
        if len(waypoint) != n_joints:
            raise ValueError(
                f"profile {profile_cfg['name']}: waypoint {waypoint} does not list one"
                f" position per joint ({n_joints} expected)"
            )

    opti = ca.Opti()
    total_time = opti.variable()
    opti.set_initial(total_time, 5.0)
    opti.subject_to(total_time > float(solver_cfg.get("min_time_seconds", 0.1)))
    opti.subject_to(total_time <= float(solver_cfg.get("max_time_seconds", 10.0)))
    dt = total_time / intervals

    positions = [opti.variable(intervals + 1) for _ in range(n_joints)]
    velocities = [opti.variable(intervals + 1) for _ in range(n_joints)]
    accelerations = [opti.variable(intervals + 1) for _ in range(n_joints)]
    jerks = [opti.variable(intervals) for _ in range(n_joints)]

    for k in range(intervals):
        for j in range(n_joints):
            opti.subject_to(positions[j][k + 1] == positions[j][k] + velocities[j][k] * dt)
            opti.subject_to(velocities[j][k + 1] == velocities[j][k] + accelerations[j][k] * dt)
            opti.subject_to(accelerations[j][k + 1] == accelerations[j][k] + jerks[j][k] * dt)

    for j, joint in enumerate(joints):
        for k in range(intervals + 1):
            opti.subject_to(positions[j][k] <= joint.max)
            opti.subject_to(positions[j][k] >= joint.min)
            opti.subject_to(velocities[j][k] <= joint.max_velocity)
            opti.subject_to(velocities[j][k] >= -joint.max_velocity)
            opti.subject_to(accelerations[j][k] <= joint.max_acceleration)
            opti.subject_to(accelerations[j][k] >= -joint.max_acceleration)
        for k in range(intervals):
            opti.subject_to(jerks[j][k] <= joint.max_jerk)
            opti.subject_to(jerks[j][k] >= -joint.max_jerk)

    # Waypoints are spread evenly over the grid; first and last are the start and goal.
    indices = [
        int(round(step * intervals / (len(waypoints) - 1))) for step in range(len(waypoints))
    ]
    for index, waypoint in zip(indices, waypoints):
        for j in range(n_joints):
            opti.subject_to(positions[j][index] == waypoint[j])

    # Point-to-point: at rest and at zero acceleration at both ends.
    for j in range(n_joints):
        opti.subject_to(velocities[j][0] == 0.0)
        opti.subject_to(accelerations[j][0] == 0.0)
        opti.subject_to(velocities[j][intervals] == 0.0)
        opti.subject_to(accelerations[j][intervals] == 0.0)

    # Regularization is solver conditioning, not behavior shaping: the default small velocity
    # weight keeps IPOPT from leaving near-zero oscillation in the solution, and the position
    # and acceleration weights default to zero so the result stays minimum-time. (The original
    # generator carried w_x = 0.1 on the elevator, which biased solutions toward staying low;
    # that is now an explicit opt-in rather than a hidden default.)
    cost = total_time
    weight_velocity = float(regularization.get("velocity", 0.01))
    weight_acceleration = float(regularization.get("acceleration", 0.0))
    weight_position = float(regularization.get("position", 0.0))
    if weight_velocity:
        cost += weight_velocity * sum(ca.sumsqr(v) for v in velocities)
    if weight_acceleration:
        cost += weight_acceleration * sum(ca.sumsqr(a) for a in accelerations)
    if weight_position:
        cost += weight_position * sum(ca.sumsqr(x) for x in positions)
    opti.minimize(cost)

    opti.solver("ipopt")
    solution = opti.solve()
    t_opt = float(solution.value(total_time))
    # sol.value() returns a DM on some casadi builds and a bare ndarray on others.
    x_solved = [as_list(solution.value(x)) for x in positions]
    v_solved = [as_list(solution.value(v)) for v in velocities]
    a_solved = [as_list(solution.value(a)) for a in accelerations]

    samples = []
    for i in range(intervals + 1):
        t = 0.0 if i == 0 else t_opt * i / intervals
        if n_joints == 1:
            samples.append(
                {
                    "t": t,
                    "x": snap(x_solved[0][i]),
                    "v": snap(v_solved[0][i]),
                    "a": snap(a_solved[0][i]),
                }
            )
        else:
            samples.append(
                {
                    "t": t,
                    "x": [snap(x_solved[j][i]) for j in range(n_joints)],
                    "v": [snap(v_solved[j][i]) for j in range(n_joints)],
                    "a": [snap(a_solved[j][i]) for j in range(n_joints)],
                }
            )
    return t_opt, samples


def snap(value):
    value = float(value)
    return 0.0 if abs(value) < NOISE_FLOOR else value


def as_list(value):
    """Flattens whatever sol.value() handed back into a plain list of floats."""
    if hasattr(value, "full"):
        value = value.full()
    return [float(entry) for entry in value]


def profile_document(profile_cfg, joints, config_path, config_notes, t_opt, samples):
    """The JSON document written per profile: metadata first, then the samples."""
    n_joints = len(joints)
    return {
        "metadata": {
            "name": profile_cfg["name"],
            "generated": datetime.date.today().isoformat(),
            "generator": GENERATOR_VERSION,
            "config": config_path.name,
            "joints": [joint.name for joint in joints],
            "units": {joint.name: joint.units for joint in joints},
            "optimal_time_seconds": round(t_opt, 6),
            "waypoints": profile_cfg["waypoints"],
            "limits": {joint.name: joint.limits() for joint in joints},
            # A profile must be auditable without the config that made it, so the config's
            # warning notes (e.g. "PLACEHOLDER limits") travel into every file.
            "notes": profile_cfg.get("notes", config_notes),
        },
        "samples": samples,
    }


def plot_profile(document, joint_names):
    """Optional inspection plot: position/velocity/acceleration per joint."""
    import matplotlib.pyplot as plt

    samples = document["samples"]
    n_joints = len(joint_names)
    figure, axes = plt.subplots(n_joints, 3, figsize=(12, 3 * n_joints), squeeze=False)
    for j, name in enumerate(joint_names):
        time = [s["t"] for s in samples]
        for column, (key, label) in enumerate(
            (("x", "position"), ("v", "velocity"), ("a", "acceleration"))
        ):
            values = [s[key][j] if isinstance(s[key], list) else s[key] for s in samples]
            axes[j][column].plot(time, values)
            axes[j][column].set_title(f"{name} {label}")
            axes[j][column].set_xlabel("time (s)")
            axes[j][column].grid(True)
    figure.tight_layout()
    plt.show()


def main():
    script_dir = Path(__file__).resolve().parent
    default_out = script_dir.parents[1] / "src" / "main" / "deploy" / "profiles"
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, required=True, help="JSON config with joints + profiles")
    parser.add_argument("--out", type=Path, default=default_out, help="output directory for profile JSONs")
    parser.add_argument("--profile", action="append", help="generate only these named profiles (repeatable)")
    parser.add_argument("--plot", help="show an inspection plot for this profile name after generating")
    args = parser.parse_args()

    config = json.loads(args.config.read_text())
    joints = [Joint(spec) for spec in config["joints"]]
    solver_cfg = config.get("solver", {})
    regularization = config.get("regularization", {})
    selected = [
        profile_cfg
        for profile_cfg in config["profiles"]
        if not args.profile or profile_cfg["name"] in args.profile
    ]
    if not selected:
        sys.exit(f"no profiles selected from {args.config}")

    args.out.mkdir(parents=True, exist_ok=True)
    joint_names = [joint.name for joint in joints]
    documents = {}
    for profile_cfg in selected:
        t_opt, samples = solve_profile(profile_cfg, joints, solver_cfg, regularization)
        document = profile_document(
            profile_cfg, joints, args.config, config.get("notes", ""), t_opt, samples
        )
        documents[profile_cfg["name"]] = document
        output_path = args.out / f"{profile_cfg['name']}.json"
        output_path.write_text(json.dumps(document, indent=2) + "\n")
        print(f"{profile_cfg['name']}: T = {t_opt:.3f} s -> {output_path}")

    if args.plot:
        if args.plot not in documents:
            sys.exit(f"--plot {args.plot} was not among the generated profiles")
        plot_profile(documents[args.plot], joint_names)


if __name__ == "__main__":
    main()
