"""Additional response/arc metrics; all existing full-motion metrics stay intact."""
import numpy as np
from .controller import desired_vector, preview_target
from .studies import sha


def fitted_time_constant(times, errors):
    """Scratch protocol: 0.02 < |e| < 0.15 m and t < 20 s, >5 samples."""
    mask = (np.abs(errors) > .02) & (np.abs(errors) < .15) & (times < 20.)
    if mask.sum() <= 5:
        return None
    slope = np.polyfit(times[mask], np.log(np.abs(errors[mask])), 1)[0]
    return float(-1. / slope) if slope < 0 else None


def angle_degrees(a, b):
    scale = np.linalg.norm(a) * np.linalg.norm(b)
    return float(np.rad2deg(np.arctan2(abs(a[0]*b[1]-a[1]*b[0]), a @ b))) if scale > 1e-20 else None


def extra_metrics(result, config, scene):
    a = result.arrays
    errors = a['signed_lateral_errors']
    m = dict(tau_fit_s=None, tau_prediction_s=None, steady_lateral_error_m=None,
             incut_prediction_m=None, incut_relative_deviation=None,
             max_abs_signed_lateral_error_m=float(np.abs(errors).max()),
             goal_switch_time_s=None, goal_switch_direction_change_deg=None,
             goal_switch_same_pose_change_deg=None,
             stopping_distance_bound_m=config.translation_speed**2/(2*config.ay))
    if scene['kind'] == 'offset':
        m['tau_fit_s'] = fitted_time_constant(a['times'], errors)
        if config.fixed_lookahead is not None:
            m['tau_prediction_s'] = (config.response_distance or config.fixed_lookahead) / config.translation_speed
    if scene['kind'] == 'arc':
        times = a['times']
        # Exact time average over the middle temporal third, interpolating edges.
        start, end = times[-1]/3, 2*times[-1]/3
        t = np.r_[start, times[(times > start) & (times < end)], end]
        steady = float(np.trapezoid(np.interp(t, times, errors), t)/(end-start))
        prediction = config.fixed_lookahead * (config.response_distance or config.fixed_lookahead)/(2*scene['radius'])
        m.update(steady_lateral_error_m=steady, incut_prediction_m=prediction,
                 incut_relative_deviation=(steady-prediction)/prediction)
    rules = a['response_rules']
    switches = np.flatnonzero((rules == 3) & np.r_[True, rules[:-1] != 3])
    if len(switches):
        i = int(switches[0])
        m['goal_switch_time_s'] = float(a['times'][i])
        if i > 0:
            m['goal_switch_direction_change_deg'] = angle_degrees(a['demands'][i-1, :2], a['demands'][i, :2])
        # Deterministic diagnostics only: noisy observations are not reconstructed.
        if config.noise_xy == config.noise_yaw == 0:
            current = a['applied'][i-1] if i else np.zeros(3)
            target, _, _, _ = preview_target(a['poses'][i], current, a['path'], config)
            before = desired_vector(a['poses'][i], target, config, a['path'], goal_region=False)
            m['goal_switch_same_pose_change_deg'] = angle_degrees(before[:2], a['demands'][i, :2])
    return m


def comparison_key(spec):
    """Simulation inputs shared by v2/v3, independent of labels and source hashes."""
    return sha({k: ({name: value for name, value in v.items() if name != 'response_distance'}
                    if k == 'config' else v)
                for k, v in spec.items() if k not in ('numerical_source_sha256', 'scenario', 'evaluation_end')})
