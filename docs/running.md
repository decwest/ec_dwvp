# Running simulations

Install the locked environment with `uv sync --locked --extra dev`.
The command-line entry point is `ec-dwvp-study`. The previous `dwvp-study`
entry point remains available for existing scripts.

## Study profiles

`configs/access_v3.yaml` includes lateral-response tuning, orientation changes,
constraint sweeps, speed regulation, and localization noise. Run a selected study
or the complete configured set:

```bash
uv run ec-dwvp-study test1 --config configs/access_v3.yaml --output results/local --seed 0
uv run ec-dwvp-study all --config configs/access_v3.yaml --output results/local --seed 0 --workers 4
```

`test1` through `test5` select individual groups. The full set includes parameter
sweeps and repeated noise realizations, so it takes longer than an individual group.
`--workers` controls independent simulation processes.

Earlier profiles remain available in `configs/access_v2.yaml` and
`configs/paper.yaml`. The `mechanism`, `obstacles`, `sweeps`, and `legacy-all`
commands use the legacy runner. Select the profile explicitly when comparing runs.

## Controller settings

The configuration specifies reference speed, lookahead, velocity and acceleration
bounds, controller rate, terminal tolerances, and optional speed regulation.
Positive `response_time` enables lateral compensation and is converted to
`response_distance = response_time * translation_speed` when the configuration
is initialized. Specify either `response_time` or `response_distance`, not both.
A null response distance retains the original VP desired velocity.

Python use with compensation enabled:

```python
from ec_dwvp.config import Config
from ec_dwvp.paths import straight_path
from ec_dwvp.simulation import simulate

config = Config(response_time=0.75)
result = simulate(straight_path(), "dwvp", config, initial_pose=[0.0, 0.5, 0.0])
```

Method keys and stored configuration fields retain their existing meanings.
`dwvp` selects commands geometrically within the dynamic window, while `vp` and
`vp_scaled` select the clipping and uniform-scaling comparisons. Compensation
is controlled separately by the response setting.

## Model and outputs

The plant integrates body-frame motion in SE(2), with per-axis velocity and
acceleration constraints. Configured noise perturbs the pose seen by the controller.
Reference yaw is supplied independently of the path tangent.

Outputs include summary CSVs, plots, run manifests, and per-trial JSON/NPZ data.
The manifest records settings, seeds, source hashes, dependency versions, and
outcomes. Timed-out trials are retained. Timing measurements depend on the host.

A cached trajectory is reused only when its complete input and source fingerprint
match. Use `--force` to recompute a matching run. Keep existing result directories
when comparing revisions, because source changes create different fingerprints.

## Verification

`uv run pytest -q` checks the simulator without ROS. Shared numerical fixtures
also support checking a built Nav2 solver runner:

```bash
uv run python tools/check_plugin_parity.py /path/to/solver_fixture_runner
```

Saved result sets can be inspected with the corresponding validation tool in
`tools/`. These tools check simulation outputs and do not exercise a physical robot.
