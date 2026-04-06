# Contributing

## Pre-merge checklist

1. **Build** — From a clean or incremental tree: `cd ros2_ws`, `source /opt/ros/jazzy/setup.bash`, `colcon build --symlink-install`, `source install/setup.bash`. Fix any new `package.xml` or `setup.py` install errors.
2. **Smoke script** — Run `./scripts/smoke_workspace.sh` from the repository root (requires sourced workspace or Docker image with workspace built). It checks packages and launch argument introspection only; it does not start Gazebo. If your machine has no ROS install, use the Docker one-liner in the script header.
3. **CI** — Pushes and pull requests to `main`, `master`, and `competition-sim` run `.github/workflows/ros2-ci.yml`: Docker image build, full `colcon build`, the smoke script, and `colcon test --packages-select luna_mapping`.
4. **Sim (manual gate)** — Before merging changes that touch launches, Nav2, mapping, or mission logic, run the flow in [COMPETITION_SIM.md](COMPETITION_SIM.md) on a machine with GPU/display or the project Docker setup. Record pass or fail in the PR.
5. **Docs** — If TF, frames, primary launches, or parameter ownership change, update [ARCHITECTURE.md](ARCHITECTURE.md) (including **Configuration index** and **Repository inventory** there). Do not duplicate long parameter tables; link to YAML files.
6. **Style** — Match surrounding code. Avoid drive-by refactors unrelated to the PR. Keep debug logging behind parameters defaulting to off.

## Tests

Packages that define `colcon test` (for example linters under `luna_mapping`) should keep passing. `luna_mapping` uses [`ament_flake8.ini`](../ros2_ws/src/luna_mapping/ament_flake8.ini) and explicit `pep257` ignores in [`test/test_pep257.py`](../ros2_ws/src/luna_mapping/test/test_pep257.py) so legacy layout does not block merges; tighten these when cleaning style.

```bash
cd ros2_ws
colcon test --packages-select luna_mapping
colcon test-result --verbose
```

Extend tests only when the cost is low (lint, small launch graph checks). Full stack simulation in CI without GPU is optional; document manual sim validation instead of claiming CI covers it.

## Submodules

`luna_ros2_worlds`, `teleop_twist_keyboard`, and `twist_stamper` are submodules. Prefer upstream alignment or pin with a note in [ARCHITECTURE.md](ARCHITECTURE.md) (Repository inventory).
