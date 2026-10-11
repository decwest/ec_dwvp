# Scope of the generated evidence

The canonical suite has 248 condition entries, corresponding to 204 distinct deterministic trajectories. Nominal settings reused across studies are not independent samples. All requested settings completed the common settled-goal criterion within 120 seconds in this run; all final commands obeyed the physical velocity and acceleration box, and neither controller collided in the specified obstacle scene.

DWVP has smaller normalized direction distortion than adaptive VP clipping on the four reference paths. For the independent-heading cosine path, both mean position and heading errors are smaller with DWVP, with a longer travel time. On the constant-heading corner, adaptive VP has the smaller mean position error. Thus the data do not support a universal tracking-error or speed advantage.

In the obstacle scene, cost regulation reduces speed near the obstacles, and combining cost and approach regulation lengthens the run. Clearance does not universally improve: the slower path can follow the reference more closely and approach a side obstacle more closely. Both methods already avoid collision without this regulation in the prescribed scene. These results demonstrate that the added cap affects speed while retaining physical feasibility, not a new obstacle-avoidance capability or a safety guarantee.

Eight sweep entries temporarily exceed the newly requested regulated speed cap, with a maximum excess of 0.113 m/s, because the cap contracts faster than physically feasible deceleration. This is the documented reachable-endpoint fallback: physical velocity and acceleration limits still hold. No canonical mechanism or obstacle entry exceeds its requested cap beyond 1e-10 m/s.

All sweep rows are retained. Figures show error and duration against the varied parameter, with unsuccessful conditions marked if present. A separate small unit-test timeout verifies failure retention; that synthetic test is not counted among publication conditions. Timing measures the Python implementation on the recorded host and is not evidence that the ROS controller always meets its deadline.

The run is noise-free and uses an ideal body-velocity plant with a circular footprint. There are no confidence intervals or statistical significance claims from deterministic repetition. New HSR experiments, wheel-level constraints, actuator lag, localization uncertainty, obstacle layouts beyond this fixed scene, and live Nav2 navigation are outside the generated simulation evidence. Actual values, configurations and code fingerprints are in `results/paper/manifest.json` and the complete CSV/JSON summaries.

## Response-distance study (`access_v3`)

The five-test study is recorded in [the v3 report](../results/access_v3/REPORT.md),
[manifest](../results/access_v3/manifest.json), per-test CSVs and PDF/PNG figures.
It contains 2641 condition entries and 2508 distinct trajectories: 2579 successes,
62 timeouts and no simulation errors or collisions. The timeouts comprise two
0.1-scale travel-time-search candidates, all 20 DWPP offset seeds at each of
0.01 and 0.02 m noise, and all 20 null-base DWVP offset seeds at 0.02 m noise
with fixed 0.055 m preview. They remain in the summaries; failed travel times
are undefined, and recovery requires all 20 seeds to succeed.

For the compensated DWVP offset runs, settling time is independent of the three
sampled lookahead distances at each fixed response distance. At `D_y` = 0.055,
0.11, 0.165 and 0.33 m, the respective 2% settling times are 3.700, 3.200,
4.067 and 6.467 s. Crossing occurs only for 0.055 m (0.018437 m), below the
0.11 m stopping-distance comparison bound. The requested fitted constants at
`L=0.11 m`, `D_y=0.165,0.33 m` are 0.727405 and 1.483271 s, within 5% of
0.75 and 1.5 s. This is a finite simulation grid, not a general bound on
crossing or a closed-loop stability proof.

Clipped VP uses the same compensated demand but changes the transient: at
`D_y=0.055 m` it settles 0.7 s earlier and crosses less than DWVP; at the other
three response distances it settles 0.367–0.500 s later and accumulates more
position error. Thus these data do not imply a universal advantage for DWVP.

On the CCW radius-1 m arc, the signed middle-third offsets differ from
`L*D_y/(2R)` (or `L²/(2R)` for the null base) by −0.916% to +2.270%.
All 30 Test 1 conditions and all 30 Test 2 conditions reach the settled goal.
For `L=0.11 m, D_y=0.33 m`, the goal preview rule starts at 20.7 s and is used
for 76 command cycles before five terminal cycles; the successive-demand
translation direction changes by 0.001165205°, and the same-pose rule change
is 0.001167608°.

The smallest sampled response distances recovering travel time to within 10%
of their own noise-free mean are 0.165, 0.165, 0.165 and 0.44 m for position
noise 0, 0.002, 0.005 and 0.01 m (with the paired yaw noise in the profile).
No sampled distance through 0.60 m meets this threshold at 0.02 m noise.
The corresponding null-base lookaheads are 0.055, 0.055, 0.165 and 0.33 m;
none through 0.60 m recovers at 0.02 m noise. The estimate `30*sigma_xy` is a
velocity-jitter comparison, not a guarantee of the 10% travel-time criterion.

All common arrays and metrics in the 136 Test 3/4 conditions match saved v2
Test 2/3 data exactly; the maximum absolute array difference is zero. v2
configuration and result files are preserved. The controller's null branch
retains the original arithmetic; the active branch includes the sampled-path
tangent correction documented in [the port specification](desired_velocity.md).
The fixtures and tests distinguish interior projection identities from clamped
endpoint behavior. Validation checks full-run errors, constraints, goal and
settling criteria, rule counts, noise aggregates and seeded command replays.
These ideal-plant simulations provide no physical HSR or human-comfort evidence.
