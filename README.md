# EC-DWVP

Standalone Python simulation tools for **Error-Compensated Dynamic Window Vector
Pursuit (EC-DWVP)** on omnidirectional robots.

The simulator follows reference paths with independent position and orientation.
It supports lateral-response tuning, per-axis velocity and acceleration constraints,
and comparisons with other pursuit controllers. ROS is not required.

The ROS 2 implementation is available in
[nav2_ec_dwvp_controller](https://github.com/decwest/nav2_ec_dwvp_controller).

## Installation

Use Python 3.10 or later and [uv](https://docs.astral.sh/uv/):

```bash
git clone https://github.com/decwest/ec_dwvp.git
cd ec_dwvp
uv sync --locked --extra dev
```

The bundled plot styles use Times New Roman. On Ubuntu, install
`ttf-mscorefonts-installer` before generating figures.

## Run

Run the lateral-response examples:

```bash
uv run ec-dwvp-study test1 --config configs/access_v3.yaml --output results/local --seed 0 --workers 4
```

The command writes configuration metadata, numerical summaries, trajectory data,
and plots to the selected output directory. Use a separate directory for local runs.

```bash
uv run ec-dwvp-study --help
```

See [running simulations](docs/running.md) for configuration, other study commands,
and output formats. The configurations include both compensated and uncompensated
controllers, with every comparison condition retained.

## Testing

```bash
uv run pytest -q
```

Tests cover velocity and acceleration limits, geometric command selection,
error compensation, reference paths, and numerical agreement with shared fixtures.

## License

[MIT](LICENSE). Source attribution is in [docs/provenance.md](docs/provenance.md).
