# Interpreting simulation outputs

Each run records its inputs and outcomes in the output directory. Use its manifest
together with the configuration and summary CSVs when comparing conditions.

- Position error is measured against the closest reference-path segment.
- Orientation error uses the reference yaw interpolated on that segment.
- Travel and settling times follow the criteria in the selected study profile.
- A timeout remains in the results and is not a successful goal arrival.
- Repeated deterministic settings are not independent noisy trials.
- Wall-clock computation time depends on the host and is recorded separately.

The simulator distinguishes desired velocities, selected commands, and velocities
applied by the ideal plant. Pose noise, when configured, changes the controller's
observation. The plant has no wheel-level dynamics or actuator lag.

Saved result directories include source and configuration metadata. Keep those
records unchanged when running a newer revision, and choose a separate output
directory for the new run. See [running simulations](running.md) for commands
and validation tools.
