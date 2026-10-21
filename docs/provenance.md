# Source attribution

The simulation package is distributed under the [MIT license](../LICENSE).

- The original ray/box solver and path generators derive from `decwest/dwpp`,
  branch `dwvp`, commit `28bc0207`.
- The package organization was informed by `decwest/dwpp_python`, commit
  `4719f452f1d44c57f3a07d1fc8f69a5db3e17412`.
- The forward differential-drive reference in `dwpp.py` uses
  `optimal_velocity_in_window` from `decwest/dwpp`, commit
  `6e6fa05b02749b166aaaa342a05efd4989c36d52`, `simulator/controllers.py`.
  The original Fumiya Ohnishi copyright and MIT license are retained.

Shared fixtures in `fixtures/` record the source revision and numerical inputs.
They check the solver, desired-velocity calculation, and differential-drive
reference against their source implementations. Fixture generation tools are in
`tools/`.

The corresponding ROS 2 plugin is
[nav2_ec_dwvp_controller](https://github.com/decwest/nav2_ec_dwvp_controller).
Its shared-fixture tests check numerical agreement for the specified cases.

Run manifests record the source hash, repository revision, configuration, path,
seed, interpreter, and dependency versions. Existing result files retain their
original provenance when the package is renamed.
