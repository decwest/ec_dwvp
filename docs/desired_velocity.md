# Desired velocity and response-distance parity contract

The simulator accepts `Config.response_distance` in metres: `None` preserves the original VP law exactly, and a finite positive value enables error compensation for `dwvp`, `vp`, `vp_scaled`, and `vp_scaled_accel`. `dwpp` and `rpp` ignore this parameter. The feasible boxes, command selectors, speed regulation, plant and terminal control are unchanged.

Implement the following sequence in a C++ port; `fixtures/desired_vector_cases.json` supplies 192 generated cases with paths, observed poses, fixed lookahead distances, selected targets, rule labels and expected body-frame desired velocities. Regenerate them with `uv run --offline --locked --python 3.11.11 python tools/generate_desired_vector_fixtures.py` using the README cache variables.

1. **Select the preview target.** Find the earliest nearest path vertex by squared XY distance to the observed robot pose. Fixed lookahead overrides adaptive lookahead; otherwise clamp `lookahead_time * hypot(current_vx, current_vy)` to `[lookahead_min, lookahead_max]`. On the path suffix starting at that vertex, find the first pose at Euclidean distance **greater than or equal to** the lookahead distance. If none exists, select the final pose and set `goal_region=true`. Otherwise, select that outer pose and set `goal_region=false`. When an earlier retained pose exists, interpolate XY at the positive quadratic root of the circle/segment intersection, clamped to [0,1]; retain the outer pose's yaw. This is the unchanged Humble RPP preview computation in `preview_target`.
2. **Retain the original distance and angular calculation.** Let `delta = target.xy - pose.xy`, `c=cos(pose.yaw)`, `s=sin(pose.yaw)`, and `body=(c*delta.x+s*delta.y, -s*delta.x+c*delta.y)`. Compute `distance=norm(body)` and `speed=config.translation_speed`. Keep this arithmetic order even though rotation theoretically preserves the norm.
3. **Preserve the null branch literally.** If `response_distance is None`, the desired translation is `(0,0)` when `distance <= 1e-6`; otherwise it is `speed * body / distance`. Do not compute a compensated equivalent. The rule label is null (inactive).
4. **Project onto the closest segment for the compensated branch.** For every consecutive XY segment `d=b-a`, compute `q=clamp(dot(p-a,d)/dot(d,d),0,1)` when `dot(d,d)>1e-20`, and zero otherwise. Choose the earliest segment minimizing `|p-(a+q*d)|²`, including duplicate segments in this selection, exactly as `metrics.project_reference` does. Set `p_r=a+q*d`. Use `t_hat=d/|d|` for a nondegenerate segment; a duplicate inherits the nearest nonzero segment by index distance, with ties resolved toward the earlier segment. An entirely constant path uses `(1,0)`. Set `n_hat=(-t_hat.y,t_hat.x)` and `e=dot(n_hat,p-p_r)` (left positive).
5. **Construct the chord and select the response rule.** Set `chord=target.xy-p_r`, `L_prime=norm(chord)`, and `D_y=response_distance`.
   - **Goal:** when `goal_region=true`, use `D_eff=min(D_y,L_prime)` and `direction=chord/L_prime-(e/D_eff)*n_hat`. If `L_prime<1e-9`, instead use `direction=target.xy-pose.xy`. The rule is `goal` in both cases. The old zero-distance threshold from step 3 remains in effect when the goal target is within `1e-6` m.
   - **Tangent:** outside the goal region, if `L_prime<1e-9` **or the selected target is the nearest path pose**, use `direction=t_hat-(e/D_y)*n_hat` and label `tangent`.
   - **Chord:** otherwise use `direction=chord/L_prime-(e/D_y)*n_hat` and label `chord`.
6. **Normalize and rotate.** Normalize `direction` in the map frame (zero if its norm is at most `1e-12`); rotate it by minus the robot yaw using step 2's `c,s`, and multiply by `speed`. Thus the demand magnitude stays fixed while its direction changes.
7. **Compute the angular demand without modification.** Start with `T=min_orientation_time`. When `speed>1e-12`, set `T=max(T,orientation_time_weight*distance/speed)`, using the actual preview-target distance from step 2, not the chord length or `D_y`. The angular demand is `wrap(target.yaw-pose.yaw)/T`, with wrap in `[-pi,pi)`.
8. **Select the command and apply terminal control.** Use the existing regulated dynamic window and the selected method. Within the position goal tolerance, the existing terminal law replaces the entire demand; its `response_rule` is null. `Command.response_rule` is also null for the differential-drive references. The goal **preview** region and the terminal **control** region are distinct.

## Sampled-path correction to the brief

The brief assumes that selecting the nearest preview pose makes the chord vanish when `|e| >= L`. A nearest vertex generally differs from the continuous closest-segment projection. Its short chord can point backward and alternate direction every cycle. With the literal `L_prime<1e-9` fallback alone, the requested straight-path cases at `L=0.11` and `D_y=0.165,0.33` took 76.267 and 93.500 s, with no fitting samples before 20 s. The additional nearest-target tangent case implements the intended absence of a forward preview segment; it preserves the chosen target and angular calculation. This correction applies only to the non-null branch and is covered by the parity fixture and unit tests.

## Compensation identity and endpoint qualification

For an interior orthogonal projection, `p-p_r=e*n_hat`, and the normalized forms are identical:

```text
u = unit((p_L-p_r)/L_prime - (e/D_y)*n_hat)
  = unit((p_L-p)/L_prime + (1/L_prime-1/D_y)*e*n_hat).
```

At `D_y=L_prime`, the compensation vanishes. At a clamped segment endpoint, `p-p_r` can also contain a tangential component, so the displayed identity is not an unrestricted endpoint identity. The implementation uses the specified closed form and the explicit goal/tangent fallbacks. Random identity tests use interior projections; endpoint cases test their specified rules directly. The null branch always aims at the selected target, including endpoints, and does not rely on this identity.

## Saved evidence

`trajectory.npz` includes `response_rules` with codes `0=inactive`, `1=chord`, `2=tangent`, `3=goal`, one value per command cycle. `trial.json` and each summary row include `response_{inactive,chord,tangent,goal}_steps`; their sum equals the number of applied command cycles. Inactive includes terminal cycles and every cycle of the null branch and differential references. `signed_lateral_errors` has one value per true pose, including both initial and final poses, using the same projection and left normal as the controller. Noise perturbs only the controller's observed pose.

The first goal-rule transition has two diagnostics: the direction difference between consecutive body-frame translation demands, and (for noise-free runs) the difference between the unconstrained chord rule and the goal rule at the same pose and preview target. Undefined zero-vector directions remain null. Tests use fixed yaw in the offset scene, so body- and map-frame differences there coincide.
