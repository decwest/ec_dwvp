# access_v3 results

4241 conditions; 4028 distinct trajectories; 4159 successes, 82 timeouts, 0 errors/collisions.

All errors cover start through settled goal arrival or timeout. Failed travel times and 2% settling times are undefined; noise recovery requires all 20 seeds to succeed. Complete per-condition metrics remain in each test’s `summary.csv`.

Signed lateral error uses the closest segment’s left normal. For straight–arc–straight paths, entry_incut_m is the maximum signed error over projected arclength entry ≤ s ≤ exit; exit_bulge_m is the minimum for s > exit. Positive error is inside the CCW arc; negative error after exit is outside. Arclength is reconstructed from reference_segment and reference_fraction on the sampled polyline, not robot travel distance.

entry_reaction_m is entry minus the projected arclength of the first saved sample before entry with signed error > 0.001 m, or zero if none exists. exit_tau_fit_s is −1/slope from a least-squares fit of log|e| versus time, using exit ≤ s < polyline length − approach_distance (0.6 m here) and 5–90% of the error magnitude at the first saved sample at/after exit. Fits with at most five samples, zero reference error or nonnegative slope are undefined; neither metric interpolates threshold crossings.

| test | scene | method | parameter | value | σxy [m] | σyaw [deg] | status | count |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| test5 | rapid | vp | match_search | 0.100 | 0.000 | 0.000 | timeout | 1 |
| test5 | rapid | vp_scaled | match_search | 0.100 | 0.000 | 0.000 | timeout | 1 |
| test5 | offset | dwpp | adaptive | — | 0.010 | 0.500 | timeout | 20 |
| test5 | offset | dwvp | fixed_lookahead | 0.055 | 0.020 | 1.000 | timeout | 20 |
| test5 | offset | dwpp | adaptive | — | 0.020 | 1.000 | timeout | 20 |
| test5 | offset | dwvp | fixed_lookahead | 0.055 | 0.020 | 0.000 | timeout | 20 |

## Test 1: lateral response

DWVP results below; clipped VP and DWPP comparisons and all full-motion metrics are in [test1/summary.csv](test1/summary.csv). τ fit uses 0.02 < |e| < 0.15 m and t < 20 s. The stopping-distance comparison v²/(2ay) is 0.110 m; it is not a closed-loop guarantee.

| L [m] | Dy [m] | τy [s] | τ fit [s] | 2% [s] | cross [m] | travel [s] | goal |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.110 | 0.055 | 0.250 | 0.351 | 3.700 | 0.018437 | 25.067 | yes |
| 0.110 | 0.110 | 0.500 | 0.420 | 3.200 | 0.000000 | 24.267 | yes |
| 0.110 | 0.165 | 0.750 | 0.727 | 4.067 | 0.000000 | 23.933 | yes |
| 0.110 | 0.330 | 1.500 | 1.483 | 6.467 | 0.000000 | 23.400 | yes |
| 0.110 | base | 0.500 | 0.383 | 3.000 | 0.000000 | 25.500 | yes |
| 0.165 | 0.055 | 0.250 | 0.351 | 3.700 | 0.018437 | 25.067 | yes |
| 0.165 | 0.110 | 0.500 | 0.420 | 3.200 | 0.000000 | 24.267 | yes |
| 0.165 | 0.165 | 0.750 | 0.727 | 4.067 | 0.000000 | 23.933 | yes |
| 0.165 | 0.330 | 1.500 | 1.483 | 6.467 | 0.000000 | 23.400 | yes |
| 0.165 | base | 0.750 | 0.757 | 4.100 | 0.000000 | 25.267 | yes |
| 0.330 | 0.055 | 0.250 | 0.351 | 3.700 | 0.018437 | 25.067 | yes |
| 0.330 | 0.110 | 0.500 | 0.420 | 3.200 | 0.000000 | 24.267 | yes |
| 0.330 | 0.165 | 0.750 | 0.727 | 4.067 | 0.000000 | 23.933 | yes |
| 0.330 | 0.330 | 1.500 | 1.483 | 6.467 | 0.000000 | 23.400 | yes |
| 0.330 | base | 1.500 | 1.453 | 6.433 | 0.000000 | 24.500 | yes |

At Dy=0.055 m, settling across sampled L spans 3.700–3.700 s.
At Dy=0.11 m, settling across sampled L spans 3.200–3.200 s.
At Dy=0.165 m, settling across sampled L spans 4.067–4.067 s.
At Dy=0.33 m, settling across sampled L spans 6.467–6.467 s.

Base τy is the small-error value L/v; DWPP fits are descriptive. Fixed-Dy independence of L is an observation on this finite grid.

## Test 2(a): orientation preview

| L [m] | Dy [m] | τy [s] | eθ,max [deg] | travel [s] | goal |
| --- | --- | --- | --- | --- | --- |
| 0.055 | 0.110 | 0.500 | 5.510 | 24.900 | yes |
| 0.055 | 0.330 | 1.500 | 5.508 | 24.900 | yes |
| 0.055 | base | 0.250 | 5.514 | 24.900 | yes |
| 0.080 | 0.110 | 0.500 | 7.769 | 24.433 | yes |
| 0.080 | 0.330 | 1.500 | 7.772 | 24.433 | yes |
| 0.080 | base | 0.364 | 7.768 | 24.433 | yes |
| 0.110 | 0.110 | 0.500 | 10.555 | 24.233 | yes |
| 0.110 | 0.330 | 1.500 | 10.548 | 24.233 | yes |
| 0.110 | base | 0.500 | 10.555 | 24.233 | yes |
| 0.165 | 0.110 | 0.500 | 17.541 | 23.867 | yes |
| 0.165 | 0.330 | 1.500 | 17.540 | 23.867 | yes |
| 0.165 | base | 0.750 | 17.540 | 23.867 | yes |
| 0.220 | 0.110 | 0.500 | 24.174 | 23.567 | yes |
| 0.220 | 0.330 | 1.500 | 24.174 | 23.567 | yes |
| 0.220 | base | 1.000 | 24.174 | 23.567 | yes |
| 0.330 | 0.110 | 0.500 | 35.769 | 23.100 | yes |
| 0.330 | 0.330 | 1.500 | 35.781 | 23.100 | yes |
| 0.330 | base | 1.500 | 35.781 | 23.100 | yes |

## Test 2(b): entry into a CCW arc

The reference starts at (0, 0) with zero pose error: a +x straight of 2 m, a CCW 90° arc of radius 1 m, and an outgoing tangent straight of 2 m. There are no transition curves. Tangent heading is continuous; the analytic arclength spacing is 0.005 m with the endpoint included.

Entry and exit arclengths are 2 m and 3.570796327 m. The entry and exit extrema use the projected-arclength intervals defined above, over the full run. max |e| and eθ,max also cover the full run. Base uses Dy=L′; its tabulated τy=L/v is a small-error label.

| L [m] | Dy [m] | τy [s] | entry_reaction_m | exit_tau_fit_s | entry_incut_m | exit_bulge_m | max \|e\| [m] | eθ,max [deg] | travel [s] | goal |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.110 | 0.110 | 0.500 | 0.016343 | 0.498724 | 0.006062 | 2.23022e-11 | 0.006062 | 2.274 | 30.333 | yes |
| 0.110 | 0.165 | 0.750 | 0.016344 | 0.743320 | 0.009087 | 1.12836e-08 | 0.009087 | 2.274 | 30.333 | yes |
| 0.110 | 0.330 | 1.500 | 0.016346 | 1.486949 | 0.017885 | 9.06661e-06 | 0.017885 | 2.274 | 30.267 | yes |
| 0.110 | L′ (base) | 0.500 | 0.016343 | 0.498674 | 0.006071 | 2.2315e-11 | 0.006071 | 2.274 | 30.333 | yes |
| 0.165 | 0.110 | 0.500 | 0.053010 | 0.504840 | 0.009105 | 1.35692e-10 | 0.009105 | 3.434 | 30.333 | yes |
| 0.165 | 0.165 | 0.750 | 0.053011 | 0.749412 | 0.013658 | 1.65636e-08 | 0.013658 | 3.434 | 30.300 | yes |
| 0.165 | 0.330 | 1.500 | 0.060343 | 1.490845 | 0.026903 | 9.8957e-06 | 0.026903 | 3.471 | 30.233 | yes |
| 0.165 | L′ (base) | 0.750 | 0.053011 | 0.749248 | 0.013706 | 1.65788e-08 | 0.013706 | 3.434 | 30.300 | yes |
| 0.330 | 0.110 | 0.500 | 0.192339 | 0.512025 | 0.018286 | -2.01231e-08 | 0.018286 | 6.942 | 30.267 | yes |
| 0.330 | 0.165 | 0.750 | 0.192341 | 0.758766 | 0.027493 | 8.80072e-08 | 0.027493 | 6.974 | 30.200 | yes |
| 0.330 | 0.330 | 1.500 | 0.199674 | 1.501632 | 0.054218 | 1.51323e-05 | 0.054218 | 7.308 | 30.067 | yes |
| 0.330 | L′ (base) | 1.500 | 0.199674 | 1.500402 | 0.054929 | 1.51406e-05 | 0.054929 | 7.306 | 30.067 | yes |

LaTeX tabular fragment: [test2/test2_arc_table.tex](test2/test2_arc_table.tex).

At fixed L, the sampled 1 mm reaction distance varies by up to 0.007334 m across response settings; this threshold metric does not establish exact independence from τy.

Updated figures (PDF and PNG): `test2/test2_arc_lateral_error`, `test2/test2_incut_vs_lookahead`, and `test2/test2_arc_paths` (entry inset). The lateral-error panel uses L=0.33 m and projected arclength from 0.5 m before entry to 0.5 m before the endpoint; dotted vertical lines mark entry and exit. Orientation panels and the shared `test2_legend` retain their definitions and styling.

## Test 3: constraints

All common trajectory arrays and metrics match v2: maximum absolute difference 0 across 136 Test 3/4 conditions. Full results: [test3/summary.csv](test3/summary.csv).

| method | conditions | successes | largest eθ,max [deg] | largest command violation [%] |
| --- | --- | --- | --- | --- |
| vp | 44 | 44 | 80.336 | 0.000 |
| vp_scaled | 44 | 44 | 71.350 | 0.000 |
| dwvp | 44 | 44 | 66.204 | 0.000 |

The unchanged overshoot comparison is in [test3/overshoot_prediction.csv](test3/overshoot_prediction.csv).

## Test 4: obstacle speed regulation

All common trajectory arrays and metrics match v2: maximum absolute difference 0 across 136 Test 3/4 conditions. Full results: [test4/summary.csv](test4/summary.csv).

| regulation | method | near speed [m/s] | max ep [m] | eθ,max [deg] | travel [s] | command violation [%] |
| --- | --- | --- | --- | --- | --- | --- |
| cost=0,approach=1 | rpp | 0.220 | 0.000 | 0.000 | 23.233 | 0.000 |
| cost=0,approach=1 | dwvp | 0.220 | 0.000 | 0.000 | 23.233 | 0.000 |
| cost=1,approach=1 | rpp | 0.165 | 0.000 | 0.000 | 25.967 | 0.000 |
| cost=1,approach=1 | dwvp | 0.165 | 0.000 | 0.000 | 25.967 | 0.000 |

## Test 5: matched travel time and noise

| method | scale | travel [s] | eθ,max [deg] | max ep [m] |
| --- | --- | --- | --- | --- |
| dwvp | 1.000 | 24.033 | 13.857 | 0.002 |
| vp | 0.961 | 24.033 | 32.585 | 0.001 |
| vp_scaled | 0.996 | 24.033 | 11.246 | 0.003 |

The unchanged v2 travel-time search scales x/y velocity limits, translation demand and nominal speed together; acceleration limits remain fixed. All noise means, sample SDs, counts and failures are retained in [test5/noise_summary.csv](test5/noise_summary.csv). Zero-noise seeds repeat deterministic trajectories.

Recovery below means all 20 seeds succeed and mean travel time is at most 1.1 times the mean at the same noise-free setting. Empty values mean no sampled setting qualifies. The comparison estimate is 30 σxy [m], not a guarantee.

| noise component | parameter | σxy [m] | σyaw [deg] | estimate [m] | smallest within 10% [m] | travel [s] | paired zero [s] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| combined | response_distance | 0.002 | 0.100 | 0.060 | 0.165 | 24.013 | 23.933 |
| combined | response_distance | 0.005 | 0.200 | 0.150 | 0.165 | 24.702 | 23.933 |
| combined | response_distance | 0.010 | 0.500 | 0.300 | 0.440 | 25.245 | 23.267 |
| combined | response_distance | 0.020 | 1.000 | 0.600 | — | — | — |
| combined | fixed_lookahead | 0.002 | 0.100 | 0.060 | 0.055 | 28.150 | 26.867 |
| combined | fixed_lookahead | 0.005 | 0.200 | 0.150 | 0.165 | 26.005 | 25.267 |
| combined | fixed_lookahead | 0.010 | 0.500 | 0.300 | 0.330 | 25.735 | 24.500 |
| combined | fixed_lookahead | 0.020 | 1.000 | 0.600 | — | — | — |
| xy_only | response_distance | 0.002 | 0.000 | 0.060 | 0.165 | 24.002 | 23.933 |
| xy_only | response_distance | 0.005 | 0.000 | 0.150 | 0.165 | 24.603 | 23.933 |
| xy_only | response_distance | 0.010 | 0.000 | 0.300 | 0.220 | 25.745 | 23.700 |
| xy_only | response_distance | 0.020 | 0.000 | 0.600 | 0.600 | 25.475 | 23.233 |
| xy_only | fixed_lookahead | 0.002 | 0.000 | 0.060 | 0.055 | 27.917 | 26.867 |
| xy_only | fixed_lookahead | 0.005 | 0.000 | 0.150 | 0.110 | 27.878 | 25.500 |
| xy_only | fixed_lookahead | 0.010 | 0.000 | 0.300 | 0.220 | 27.333 | 25.000 |
| xy_only | fixed_lookahead | 0.020 | 0.000 | 0.600 | — | — | — |

| σyaw [deg] | min noisy/zero travel-time ratio | max ratio |
| --- | --- | --- |
| 0.100 | 1.000 | 1.000 |
| 0.200 | 1.001 | 1.002 |
| 0.500 | 1.021 | 1.023 |
| 1.000 | 1.189 | 1.225 |

Detailed recovery and yaw-only ratios: [combined](test5/travel_time_recovery.csv), [position-only](test5/travel_time_recovery_xy_only.csv), [yaw-only](test5/yaw_only_ratios.csv). Test 5(c) rapid-ramp fixed-preview results remain in [test5/summary.csv](test5/summary.csv) and `test5/test5_orientation_vs_lookahead.{pdf,png}`.

## Verification and provenance

| method | distinct trials | cycle-weighted mean [µs] | median trial p95 [µs] | maximum [µs] |
| --- | --- | --- | --- | --- |
| dwvp | 3201 | 154.695 | 190.354 | 5393.200 |
| vp | 367 | 80.633 | 112.743 | 1955.970 |
| vp_scaled | 355 | 84.535 | 122.005 | 890.759 |
| dwpp | 103 | 82.784 | 130.607 | 2935.271 |
| rpp | 2 | 83.738 | 142.046 | 375.484 |

Times measure Python preview, demand and command calculation, excluding noise generation, integration and plotting; they are not a ROS real-time guarantee.

Current simulation/cache wall time: 117.1 s; 213 cache hits; 8 workers. Earlier attempts remain in the manifest.

Current run including figure regeneration: 128.6 s.

Immediately preceding run: simulation/cache 17.6 s; including figures 29.4 s. Complete timing history is in the manifest.

Validation: passed; 4241 conditions, 4028 trajectories; runtime 168.5 s; 17251 protected files unchanged.

Pytest: 117 passed in 4.11s

Source SHA-256: `ce65edd69c52368f653f48491fab4419a1a60564f4eb1b0ee7bd4d6701681734`; numerical source: `2605e77c896500abc93d22a6a16f2422ef64830189aecfe8797253345410010f`.

All 85 PDF/PNG figure pairs are listed in [manifest.json](manifest.json). Per-trial histories under `trials/` are Git-ignored. The compensation identity and endpoint limits are in `docs/desired_velocity.md`.

Changed source, configuration, documentation and tests: `docs/results.md`, `src/omnidirectional_dwvp/access_v3_metrics.py`, `src/omnidirectional_dwvp/access_v3_outputs.py`, `tests/test_access_v3.py`.
