"""Figures, compact tables and v2 parity evidence for access_v3."""
import json
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from .access_metrics import aggregate
from .access_noise import noise_lookahead_estimate
from .access_plotting import style, save, figures as v2_figures, COLORS, STYLES, LABELS
from .access_report import prediction_comparison
from .access_studies import atomic_json
from .access_v3_metrics import comparison_key
from .config import Config
from .studies import ROOT, write_csv

RESPONSE_COLORS = {None: '#333333', .055: '#a5509a', .11: '#2568a0', .165: '#23845d', .33: '#bf4145'}
NOISE_COLORS = ('#333333', '#2568a0', '#23845d', '#b87510', '#bf4145')
NOISE_KEYS = ('scenario', 'method', 'parameter', 'value', 'fixed_lookahead_m', 'response_distance_m', 'tau_y_s', 'noise_xy_m', 'noise_yaw_deg')


def tag(value):
    return f'{value:g}'.replace('.', 'p')


def label(response, speed, distance=False):
    if response is None:
        return r"$D_y=L'$"
    result = rf'$\tau_y={response/speed:g}$ s'
    return result + (rf' ($D_y={response:g}$ m)' if distance else '')


def load(output, row):
    with np.load(output/'trials'/row['trial_id']/'trajectory.npz') as saved:
        return {k: saved[k] for k in saved}


def finish(fig, ax, stem, xlabel, ylabel, legend=True):
    ax.set(xlabel=xlabel, ylabel=ylabel)
    ax.grid(alpha=.2, lw=.4)
    if legend:
        ax.legend(frameon=False, fontsize=7)
    save(fig, stem)


def noise_summary(rows):
    return aggregate([r for r in rows if r['test']=='test5' and r['part']=='b'], NOISE_KEYS)


def noise_component(row):
    xy, yaw = row['noise_xy_m'], row['noise_yaw_deg']
    return 'combined' if xy and yaw else 'xy_only' if xy else 'yaw_only' if yaw else 'zero'


def noise_subset(grouped, component):
    return [r for r in grouped if noise_component(r) in ('zero', component)]


def recovery_summary(grouped, config, component='combined'):
    """Paired noise-free setting; all seeds must succeed before accepting recovery."""
    result = []
    for parameter in ('response_distance', 'fixed_lookahead'):
        rows = [r for r in noise_subset(grouped, component) if r['scenario']=='offset' and r['parameter']==parameter
                and (component=='combined' or parameter=='response_distance')]
        baseline = {r['value']: r for r in rows if r['noise_xy_m']==r['noise_yaw_deg']==0}
        for xy, yaw in sorted({(r['noise_xy_m'], r['noise_yaw_deg']) for r in rows}):
            recovered = None
            candidates = sorted((r for r in rows if r['noise_xy_m']==xy and r['noise_yaw_deg']==yaw and r['value'] is not None), key=lambda r:r['value'])
            for r in candidates:
                zero = baseline[r['value']]
                if all(g['n']==g['success_count']==g['travel_time_s_n'] for g in (r, zero)) and r['travel_time_s_mean'] <= 1.1*zero['travel_time_s_mean']+1e-10:
                    recovered = r
                    break
            result.append(dict(parameter=parameter, noise_xy_m=xy, noise_yaw_deg=yaw,
                               estimate_m=noise_lookahead_estimate(config, xy),
                               estimate_tau_y_s=noise_lookahead_estimate(config, xy)/config.translation_speed,
                               minimum_distance_m=recovered['value'] if recovered else None,
                               tau_y_s=recovered['value']/config.translation_speed if recovered else None,
                               travel_time_s_mean=recovered['travel_time_s_mean'] if recovered else None,
                               zero_noise_time_s_mean=baseline[recovered['value']]['travel_time_s_mean'] if recovered else None))
    return result


def yaw_ratios(grouped):
    rows = [r for r in noise_subset(grouped, 'yaw_only') if r['scenario']=='offset' and r['parameter']=='response_distance']
    baseline = {r['value']: r for r in rows if noise_component(r)=='zero'}
    return [dict(response_distance_m=r['value'], tau_y_s=r['tau_y_s'], noise_yaw_deg=r['noise_yaw_deg'],
                 travel_time_ratio=r['travel_time_s_mean']/baseline[r['value']]['travel_time_s_mean']
                 if r['travel_time_s_mean'] is not None else None,
                 success_count=r['success_count'], n=r['n'])
            for r in rows if noise_component(r)=='yaw_only']


def noise_travel_figure(output, grouped, config, parameter, component='combined'):
    group = [r for r in noise_subset(grouped, component) if r['scenario']=='offset' and r['parameter']==parameter]
    if not group:
        return
    response = parameter=='response_distance'
    if response:
        fig, (ax, base) = plt.subplots(1, 2, figsize=(6.1, 2.9), sharey=True, gridspec_kw={'width_ratios': [4.5, 1]})
    else:
        fig, ax = plt.subplots(figsize=(4.3, 2.7))
        base = None
    levels = sorted({(r['noise_xy_m'], r['noise_yaw_deg']) for r in group})
    for i, (xy, yaw) in enumerate(levels):
        series = sorted([r for r in group if (r['noise_xy_m'],r['noise_yaw_deg'])==(xy,yaw) and r['value'] is not None], key=lambda r:r['value'])
        ax.errorbar([r['tau_y_s'] if response else r['value'] for r in series],
                    [r['travel_time_s_mean'] if r['travel_time_s_mean'] is not None else np.nan for r in series],
                    yerr=[r['travel_time_s_std'] or 0 for r in series], color=NOISE_COLORS[i], marker='o', ms=3, lw=1, capsize=2,
                    label=rf'$\sigma_{{xy}}={xy:g}$ m, $\sigma_\psi={yaw:g}^\circ$')
        if xy:
            estimate = noise_lookahead_estimate(config, xy)/(config.translation_speed if response else 1.)
            ax.axvline(estimate, color=NOISE_COLORS[i], ls=':', lw=.7)
            if response:
                ax.text(estimate, 1.02, f'{estimate:.2f}', transform=ax.get_xaxis_transform(), color=NOISE_COLORS[i], fontsize=6, ha='center')
        if base is not None:
            r = next(r for r in group if r['value'] is None and (r['noise_xy_m'],r['noise_yaw_deg'])==(xy,yaw))
            base.errorbar([i], [r['travel_time_s_mean'] if r['travel_time_s_mean'] is not None else np.nan],
                          yerr=[r['travel_time_s_std'] or 0], color=NOISE_COLORS[i], marker='o', ms=3, capsize=2)
    if base is not None:
        base.set(xticks=[], xlabel=r"Base $D_y=L'$", xlim=(-.8,len(levels)-.2))
        base.grid(axis='y', alpha=.2, lw=.4)
        ax.set_xlim(0, 1.05*max(r['tau_y_s'] for r in group))
        fig.legend(*ax.get_legend_handles_labels(), loc='upper center', bbox_to_anchor=(.5,-.02), ncol=2, frameon=False, fontsize=7)
    stem = 'response' if response else 'lookahead'
    suffix = '' if component=='combined' else '_'+component
    finish(fig, ax, output/'test5'/f'test5_noise_travel_time_vs_{stem}{suffix}',
           r'$\tau_y$ [s]' if response else 'Lookahead distance [m]', 'Mean travel time [s]', base is None)


class V2OutputPaths:
    """Route old plotting code directly into the corresponding v3 directories."""
    def __init__(self, output):
        self.output = output
    def __truediv__(self, key):
        return self.output / {'test2': 'test3', 'test3': 'test4'}.get(key, key)


def unchanged_figures(output, rows, manifest):
    renamed = [{**r, 'test': {'test3': 'test2', 'test4': 'test3'}[r['test']]}
               for r in rows if r['test'] in ('test3', 'test4')]
    if not renamed:
        return []
    mapped = V2OutputPaths(output)
    v2_figures(mapped, renamed, manifest['settings'], Config(**manifest['config']))
    predictions = prediction_comparison(mapped, renamed, manifest)
    for test in ('test3', 'test4'):
        for path in sorted((output/test).glob('*')):
            if path.suffix in ('.pdf', '.png') and not path.name.startswith(test+'_'):
                name = path.name.replace('max_heading_vs_length', 'max_orientation_vs_length')
                path.replace(path.with_name(test+'_'+name))
    return predictions


def figures(output, rows, manifest, grouped):
    style()
    config = Config(**manifest['config'])
    speed = config.translation_speed
    chosen = [r for r in rows if r['test']=='test1' and r['method']=='dwvp']
    if chosen:
        for look in manifest['settings']['response_lookaheads']:
            fig, ax = plt.subplots(figsize=(3.8, 2.4))
            for r in chosen:
                if r['fixed_lookahead_m'] != look:
                    continue
                a = load(output, r)
                d = r['response_distance_m']
                ax.plot(a['times'], a['signed_lateral_errors'], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', lw=1)
            ax.axhline(0, color='.5', lw=.5)
            ax.set_title(f'L = {look:g} m')
            finish(fig, ax, output/'test1'/f'test1_response_L{tag(look)}_lateral', 'Time [s]', 'Signed lateral error [m]', False)
        fig = plt.figure(figsize=(7.0, .7))
        handles = [Line2D([], [], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', label=label(d, speed, distance=True))
                   for d in [*manifest['settings']['response_distances'], None]]
        fig.legend(handles=handles, loc='center', ncol=3, frameon=False)
        save(fig, output/'test1'/'test1_response_legend')
        for metric, stem, ylabel in (('settling_2pct_time_s', 'settling', '2% settling time [s]'), ('crossing_m', 'crossing', 'Crossing [m]')):
            fig, ax = plt.subplots(figsize=(3.8, 2.4))
            for index, look in enumerate(manifest['settings']['response_lookaheads']):
                series = sorted([r for r in chosen if r['fixed_lookahead_m']==look and r['response_distance_m'] is not None], key=lambda r:r['response_distance_m'])
                ax.plot([r['tau_y_s'] for r in series], [r[metric] for r in series],
                        marker=('o','s','^')[index], ms=7-index*2, markerfacecolor='none',
                        ls=('-','--',':')[index], label=f'L={look:g} m')
            if stem=='settling':
                x = np.array(manifest['settings']['response_distances'])/speed
                ax.plot(x, x*np.log(50), ':', color='.3', label=r'$T_s=\tau_y\ln 50$')
            ax.axvline(speed/(2*config.ay), color='.5', ls=':', lw=.8, label=rf'$v_{{des}}/(2a_y)={speed/(2*config.ay):g}$ s')
            finish(fig, ax, output/'test1'/f'test1_{stem}_vs_response', r'Time constant $\tau_y$ [s]', ylabel)
    ramps = [r for r in rows if r['test']=='test2' and r['part']=='a']
    arcs = [r for r in rows if r['test']=='test2' and r['part']=='b']
    for group, stem, metric, ylabel in ((ramps, 'orientation', 'eval_max_heading_error_deg', 'Max. orientation error [deg]'),
                                       (arcs, 'incut', 'steady_lateral_error_m', 'Steady inside offset [m]')):
        if not group:
            continue
        fig, ax = plt.subplots(figsize=(3.8, 2.4))
        for d in dict.fromkeys(r['response_distance_m'] for r in group):
            series = sorted([r for r in group if r['response_distance_m']==d], key=lambda r:r['fixed_lookahead_m'])
            x = [r['fixed_lookahead_m'] for r in series]
            ax.plot(x, [r[metric] for r in series], color=RESPONSE_COLORS[d], marker='o', ms=3, label=label(d, speed), ls='--' if d is None else '-')
            if stem=='incut':
                ax.plot(x, [r['incut_prediction_m'] for r in series], color=RESPONSE_COLORS[d], ls=':', lw=.8)
        finish(fig, ax, output/'test2'/f'test2_{stem}_vs_lookahead', 'Lookahead distance [m]', ylabel)
    if arcs:
        series = [r for r in arcs if r['fixed_lookahead_m']==.33]
        fig, ax = plt.subplots(figsize=(4.5, 3.1))
        inset = ax.inset_axes([.18, .60, .45, .30])
        for i, r in enumerate(series):
            a = load(output, r)
            d = r['response_distance_m']
            if i == 0:
                for axis in (ax, inset):
                    axis.plot(a['path'][:, 0], a['path'][:, 1], ':', color='.2', label='Reference')
            for axis in (ax, inset):
                axis.plot(a['poses'][:, 0], a['poses'][:, 1], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', lw=1, label=label(d, speed))
        inset.set(xlim=(.89, 1.015), ylim=(.78, 1.03))
        inset.tick_params(labelsize=5.5)
        inset.grid(alpha=.2)
        ax.set_aspect('equal')
        ax.legend(loc='center left', bbox_to_anchor=(1.05, .5), frameon=False, fontsize=7)
        finish(fig, ax, output/'test2'/'test2_arc_paths', 'x [m]', 'y [m]', False)
    for component in ('combined', 'xy_only', 'yaw_only'):
        noise_travel_figure(output, grouped, config, 'response_distance', component)
    noise_travel_figure(output, grouped, config, 'fixed_lookahead')
    rapid = [r for r in grouped if r['scenario']=='rapid']
    if rapid:
        fig, ax = plt.subplots(figsize=(4.1, 2.6))
        for method, parameter in [('vp', 'adaptive'), ('vp_scaled', 'adaptive'), ('dwvp', 'adaptive'), ('dwvp', 'response_distance')]:
            series = sorted([r for r in rapid if r['method']==method and r['parameter']==parameter], key=lambda r:r['noise_xy_m'])
            ax.errorbar([r['noise_xy_m'] for r in series], [r['eval_max_heading_error_deg_mean'] for r in series],
                        yerr=[r['eval_max_heading_error_deg_std'] or 0 for r in series], lw=1, capsize=2,
                        color='#23845d' if parameter=='response_distance' else COLORS[method],
                        ls=':' if parameter=='response_distance' else STYLES[method], marker='o', ms=3,
                        label='DWVP L=0.11, Dy=0.33' if parameter=='response_distance' else LABELS[method])
        finish(fig, ax, output/'test5'/'test5_noise_orientation', 'Position noise standard deviation [m]', 'Mean max. orientation error [deg]')
    sweep = [r for r in rows if r['test']=='test5' and r['part']=='c']
    if sweep:
        fig, ax = plt.subplots(figsize=(3.8, 2.4))
        for method in ('vp', 'vp_scaled', 'dwvp'):
            series = sorted([r for r in sweep if r['method']==method], key=lambda r:r['value'])
            ax.plot([r['value'] for r in series], [r['eval_max_heading_error_deg'] for r in series], color=COLORS[method], ls=STYLES[method], marker='o', ms=3, label=LABELS[method])
        finish(fig, ax, output/'test5'/'test5_orientation_vs_lookahead', 'Lookahead distance [m]', 'Max. orientation error [deg]')


def parity(output, manifest):
    baseline_path = ROOT/'results/access_v2/manifest.json'
    if not baseline_path.exists():
        return dict(available=False, reason='v2 manifest absent', comparisons=[])
    baseline = json.loads(baseline_path.read_text())
    index = {comparison_key(t['spec']): t for t in baseline['trials'] if t['condition']['test'] in ('test2', 'test3')}
    comparisons = []
    for t in manifest['trials']:
        if t['condition']['test'] not in ('test3', 'test4'):
            continue
        old = index.get(comparison_key(t['spec']))
        if old is None:
            raise AssertionError(f"v2 condition missing: {t['condition_id']}")
        old_id = old['summary']['trial_id']
        maximum = 0.
        with np.load(ROOT/'results/access_v2/trials'/old_id/'trajectory.npz') as a, np.load(output/'trials'/t['summary']['trial_id']/'trajectory.npz') as b:
            for name in a.files:
                if name not in b:
                    raise AssertionError(f'missing historical array {name}')
                np.testing.assert_array_equal(a[name], b[name], err_msg=f'{old_id}: {name}')
                if np.issubdtype(a[name].dtype, np.number):
                    finite = np.isfinite(a[name]) & np.isfinite(b[name])
                    if finite.any():
                        maximum = max(maximum, float(np.abs(a[name][finite].astype(float)-b[name][finite].astype(float)).max()))
            arrays = list(a.files)
        changed = []
        for k, v in old['summary'].items():
            if k in ('test', 'part', 'condition_id', 'trial_id', 'parameter', 'value'):
                continue
            if k in t['summary'] and t['summary'][k] != v:
                changed.append(k)
        if changed:
            raise AssertionError(f'v2 metric mismatch: {changed}')
        comparisons.append(dict(condition_id=t['condition_id'], previous_trial_id=old_id,
                                method=t['condition']['method'], arrays=arrays, maximum_absolute_difference=maximum,
                                changed_common_metrics=changed))
    result = dict(available=True, compared_conditions=len(comparisons), comparisons=comparisons,
                  maximum_absolute_difference=max((r['maximum_absolute_difference'] for r in comparisons), default=0.))
    atomic_json(output/'v2_parity.json', result)
    return {k:v for k,v in result.items() if k!='comparisons'}


def fmt(value, digits=3):
    if value is None:
        return '—'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, (float, np.floating)):
        return f'{value:.{digits}f}'
    return str(value)


def report(output, rows, manifest, grouped, recovery, predictions):
    config = Config(**manifest['config'])
    original_rows = [r for r in rows if not (r['test']=='test5' and r['part']=='b' and noise_component(r) in ('xy_only','yaw_only'))]
    original_grouped = noise_subset(grouped, 'combined')
    previous_report = (output/'REPORT.md').read_text() if (output/'REPORT.md').exists() else ''
    historical_verification = ''
    if len(original_rows) != len(rows) and '## Verification and provenance\n' in previous_report:
        historical_verification = previous_report.split('## Verification and provenance\n', 1)[1].split('## Follow-up verification and provenance\n', 1)[0].strip()
    lines = ['# access_v3 results', '',
             f"{len(original_rows)} conditions; {len({r['trial_id'] for r in original_rows})} distinct trajectories; {sum(r['success'] for r in original_rows)} successes, {sum(r['status']=='timeout' for r in original_rows)} timeouts, {sum(r['status'] not in ('success','timeout') for r in original_rows)} errors/collisions.", '',
             'All errors cover start through settled goal arrival or timeout; travel time and 2% settling are undefined for failures, and noise means retain finite counts and require all seeds to succeed for recovery.', '',
             'Constraint share is the percentage of final commands violating physical velocity or acceleration bounds (v2 definition); response counts C/T/G/I denote chord/tangent/goal/inactive, with terminal control counted inactive.', '',
             'Signed lateral error uses the closest segment’s left normal; arc steady error is the time mean over the middle temporal third.', '']
    if len(original_rows) != len(rows):
        lines.extend([f'The counts above and the original sections describe the combined-noise study; the added noise-decomposition subsections and follow-up verification include {len(rows)-len(original_rows)} additional trials.', ''])
    def table(headers, data):
        cell = lambda v: fmt(v).replace('|', r'\|')
        lines.extend(['| '+' | '.join(cell(h) for h in headers)+' |', '| '+' | '.join(['---']*len(headers))+' |'])
        lines.extend('| '+' | '.join(cell(v) for v in row)+' |' for row in data)
        lines.append('')
    def heading(title):
        lines.extend(['## '+title, ''])
    failures = [r for r in original_rows if not r['success']]
    if failures:
        from collections import Counter
        counts = Counter((r['test'],r['scenario'],r['method'],r['parameter'],r['value'],r['noise_xy_m'],r['status']) for r in failures)
        table(['test','scene','method','parameter','value','noise xy [m]','status','count'],
              [[*key,count] for key,count in counts.items()])
    first = [r for r in rows if r['test']=='test1']
    if first:
        heading('Test 1: lateral response')
        lines.extend(['Stopping-distance comparison: v²/(2ay) = **0.110 m**; τ fit uses 0.02 < |e| < 0.15 m and t < 20 s.', '',
                      'For null-base rows, L/v is the small-error prediction (the chord tends to L as e tends to zero); DWPP fits are descriptive and are not predictions of a first-order omnidirectional model.', ''])
        for method in ('dwvp', 'dwpp', 'vp'):
            group = [r for r in first if r['method']==method]
            lines.extend(['### '+LABELS[method], ''])
            table(['L [m]', 'Dy [m]', 'tau_y_s', 'τ fit [s]', 'Dy/v [s]', '2% [s]', 'cross [m]', '∫|ep| [m s]', 'travel [s]', 'constraint [%]', 'C/T/G/I', 'goal'],
                  [[r['fixed_lookahead_m'], "L′" if r['response_distance_m'] is None else r['response_distance_m'],
                    r['tau_y_s'], r['tau_fit_s'], r['tau_prediction_s'], r['settling_2pct_time_s'], f"{r['crossing_m']:.6f}", r['eval_position_error_integral_m_s'], r['travel_time_s'], r['command_constraint_violation_pct'],
                    '/'.join(str(r[f'response_{rule}_steps']) for rule in ('chord','tangent','goal','inactive')), r['success']] for r in group])
        table(['method', 'L', 'Dy', 'tau_y_s', 'mean ep [m]', 'max eψ [deg]', 'mean eψ [deg]', '∫|eψ| [deg s]'],
              [[r['method'], r['fixed_lookahead_m'], r['response_distance_m'] if r['response_distance_m'] is not None else 'L′', r['tau_y_s'], r['eval_mean_position_error_m'], r['eval_max_heading_error_deg'], r['eval_mean_heading_error_deg'], r['eval_heading_error_integral_deg_s']] for r in first])
        for d in manifest['settings']['response_distances']:
            group = [r for r in first if r['method']=='dwvp' and r['response_distance_m']==d]
            settling = [r['settling_2pct_time_s'] for r in group if r['settling_2pct_time_s'] is not None]
            lines.append(f"At Dy={d:g} m, DWVP settling across L spans {min(settling):.3f}–{max(settling):.3f} s (range {max(settling)-min(settling):.3f} s)." if settling else f'At Dy={d:g} m, no finite settling estimate.')
        lines.extend(['', 'Settling has no observed dependence on L at fixed Dy in this grid.', ''])
        crossing = [r for r in first if r['method']=='dwvp' and r['response_distance_m'] is not None and r['crossed']]
        lines.extend(['', 'Compensated DWVP crossing occurs at '+(', '.join(f"(L={r['fixed_lookahead_m']:g}, Dy={r['response_distance_m']:g})" for r in crossing) or 'no sampled condition')+'; '+('all are below' if all(r['response_distance_m'] < .11 for r in crossing) else 'some are at or above')+' the 0.11 m comparison bound.', ''])
        pairs = [(a,b) for a in first for b in first if a['method']=='vp' and b['method']=='dwvp' and (a['fixed_lookahead_m'], a['response_distance_m'])==(b['fixed_lookahead_m'], b['response_distance_m'])]
        table(['L', 'Dy', 'tau_y_s', 'VP−DWVP travel [s]', 'VP−DWVP 2% [s]', 'VP−DWVP ∫|ep| [m s]', 'VP−DWVP cross [m]'],
              [[a['fixed_lookahead_m'], a['response_distance_m'], a['tau_y_s'], *[a[k]-b[k] if a[k] is not None and b[k] is not None else None for k in ('travel_time_s','settling_2pct_time_s','eval_position_error_integral_m_s','crossing_m')]] for a,b in pairs])
        switch = next(r for r in first if r['method']=='dwvp' and r['fixed_lookahead_m']==.11 and r['response_distance_m']==.33)
        lines.extend([f"At L=0.11, Dy=0.33 m, the first goal-rule switch is at {fmt(switch['goal_switch_time_s'])} s: successive-demand direction change {fmt(switch['goal_switch_direction_change_deg'],9)}°, same-pose rule change {fmt(switch['goal_switch_same_pose_change_deg'],9)}°; Test 1 has {sum(not r['success'] for r in first)} goal misses/timeouts.", ''])
    second = [r for r in rows if r['test']=='test2']
    if second:
        heading('Test 2: preview')
        table(['L [m]', 'Dy [m]', 'tau_y_s', 'max eψ [deg]', 'mean eψ [deg]', '∫|eψ| [deg s]', 'lag [deg]', 'overshoot [deg]', 'max ep [m]', 'travel [s]'],
              [[r['fixed_lookahead_m'], r['response_distance_m'] if r['response_distance_m'] is not None else 'L′', r['tau_y_s'], *[r[k] for k in ('eval_max_heading_error_deg','eval_mean_heading_error_deg','eval_heading_error_integral_deg_s','transition_heading_lag_deg','post_transition_heading_overshoot_deg','eval_max_position_error_m','travel_time_s')]] for r in second if r['part']=='a'])
        table(['L [m]', 'Dy [m]', 'tau_y_s', 'steady [m]', 'prediction [m]', 'deviation [%]', 'max |e| [m]', 'max eψ [deg]', 'mean eψ [deg]', 'travel [s]', 'goal'],
              [[r['fixed_lookahead_m'], r['response_distance_m'] if r['response_distance_m'] is not None else 'L′', r['tau_y_s'], fmt(r['steady_lateral_error_m'],6), fmt(r['incut_prediction_m'],6), 100*r['incut_relative_deviation'], r['max_abs_signed_lateral_error_m'], r['eval_max_heading_error_deg'], r['eval_mean_heading_error_deg'], r['travel_time_s'], r['success']] for r in second if r['part']=='b'])
    for test, title in (('test3','Test 3: constraints'), ('test4','Test 4: obstacle speed regulation')):
        group = [r for r in rows if r['test']==test]
        if not group:
            continue
        heading(title)
        lines.extend(['All common trajectory arrays and metrics are unchanged from v2 '+('Test 2' if test=='test3' else 'Test 3')+f"; parity maximum absolute difference is {manifest['v2_parity']['maximum_absolute_difference']:g} across {manifest['v2_parity']['compared_conditions']} total conditions.", ''])
        table(['scene / parameter / value', 'method', 'max ep [m]', 'mean ep [m]', '∫|ep| [m s]', 'max eψ [deg]', 'mean eψ [deg]', '∫|eψ| [deg s]', 'travel [s]', 'lag [deg]', 'overshoot [deg]', 'constraint [%]'],
              [[f"{r['scenario']} / {r['parameter']} / {r['value']}", r['method'], *[r.get(k) for k in ('eval_max_position_error_m','eval_mean_position_error_m','eval_position_error_integral_m_s','eval_max_heading_error_deg','eval_mean_heading_error_deg','eval_heading_error_integral_deg_s','travel_time_s','transition_heading_lag_deg','post_transition_heading_overshoot_deg','command_constraint_violation_pct')]] for r in group])
        if test=='test4':
            table(['regulation', 'method', 'near speed [m/s]', 'demand violation [%]', 'command violation [%]', 'steps', 'demand vel/acc/both', 'command vel/acc/both'],
                  [[r['value'],r['method'],r['mean_near_obstacle_speed_m_s'],r['unconstrained_demand_violation_pct'],r['command_constraint_violation_pct'],r['control_steps'],
                    '/'.join(str(r[f'demand_{k}_violation_steps']) for k in ('velocity','acceleration','both')),
                    '/'.join(str(r[f'command_{k}_violation_steps']) for k in ('velocity','acceleration','both'))] for r in group])
    if predictions:
        heading('Test 3: overshoot prediction check')
        table(['length', 'scale', 'method', 'predicted [deg]', 'observed [deg]', 'difference [deg]'],
              [[r['transition_length_m'],r['value'],r['method'],r['predicted_heading_overshoot_deg'],r['observed_heading_overshoot_deg'],r['prediction_error_deg']] for r in predictions if r['transition_length_m']==.3 and r['parameter']=='acceleration_scale'])
    fifth = [r for r in rows if r['test']=='test5']
    if fifth:
        heading('Test 5(a): matched travel time')
        selected = [r for r in fifth if r['parameter']=='match_reference' or r['trial_id'] in {m['best_trial_id'] for m in manifest['time_matches'].values()}]
        table(['method','scale','demand [m/s]','travel [s]','max eψ [deg]','∫|eψ| [deg s]', 'max ep [m]'],
              [[r['method'],r['value'],.22*r['value'],r['travel_time_s'],r['eval_max_heading_error_deg'],r['eval_heading_error_integral_deg_s'],r['eval_max_position_error_m']] for r in selected])
        lines.extend(['Matching uses the v2 search unchanged, scaling x/y velocity limits, translation demand and nominal speed together; acceleration limits remain fixed.', ''])
        heading('Test 5(b): noise')
        lines.extend(['Each cell is mean ± sample SD [s], successful/20; zero-noise seeds repeat a deterministic trajectory and all-timeout travel-time points are gaps in figures.', ''])
        for parameter in ('response_distance','fixed_lookahead'):
            group = [r for r in original_grouped if r['scenario']=='offset' and r['parameter']==parameter]
            values = list(dict.fromkeys(r['value'] for r in group))
            data = []
            for xy,yaw in manifest['settings']['noise_levels']:
                cells = []
                for v in values:
                    r = next(r for r in group if r['value']==v and r['noise_xy_m']==xy)
                    cells.extend([f"{fmt(r['travel_time_s_mean'])} ± {fmt(r['travel_time_s_std'])}, {r['success_count']}/{r['n']}", r['tau_y_s']])
                data.append([f'{xy:g} m / {yaw:g}°',*cells])
            table(['noise xy / yaw']+[h for v in values for h in (f"{parameter}={v if v is not None else 'base'}", 'tau_y_s')], data)
        table(['parameter','noise xy [m]','noise yaw [deg]','estimate [m]','estimate τy [s]','smallest within 10% [m]','tau_y_s','mean time [s]','paired zero [s]'],
              [[r[k] for k in ('parameter','noise_xy_m','noise_yaw_deg','estimate_m','estimate_tau_y_s','minimum_distance_m','tau_y_s','travel_time_s_mean','zero_noise_time_s_mean')] for r in recovery])
        lines.extend(['The recovery threshold compares each setting with its own zero-noise mean and requires all 20 seeds to succeed; the estimate is v σxy/(ay Δt) = 30 σxy.', ''])
        table(['scene','method','preview / Dy','tau_y_s','noise xy [m]','max eψ mean ± SD [deg]','travel mean [s]','success/20'],
              [[r['scenario'],r['method'], 'L=0.11 / Dy=0.33' if r['parameter']=='response_distance' else 'adaptive / base',r['tau_y_s'],r['noise_xy_m'], f"{fmt(r['eval_max_heading_error_deg_mean'])} ± {fmt(r['eval_max_heading_error_deg_std'])}",r['travel_time_s_mean'],f"{r['success_count']}/{r['n']}"] for r in grouped if r['scenario']!='offset' or r['parameter']=='adaptive'])
        for component, title in (('xy_only', 'position-only'), ('yaw_only', 'yaw-only')):
            group = [r for r in noise_subset(grouped, component) if r['scenario']=='offset' and r['parameter']=='response_distance']
            if not any(noise_component(r)==component for r in group):
                continue
            lines.extend(['### Test 5(b): '+title+' noise', '',
                          'DWVP uses L = 0.11 m and 20 seeds per setting; each cell is mean ± sample SD [s], successes/20, and zero-noise rows reuse the combined experiment.', ''])
            levels = sorted({(r['noise_xy_m'], r['noise_yaw_deg']) for r in group})
            values = manifest['settings']['noise_responses']
            table(['Dy [m]', 'tau_y_s']+[f'{xy:g} m / {yaw:g}°' for xy,yaw in levels],
                  [["L′ (base)" if value is None else value, (.11 if value is None else value)/config.translation_speed,
                    *[f"{fmt(r['travel_time_s_mean'])} ± {fmt(r['travel_time_s_std'])}, {r['success_count']}/{r['n']}"
                      for xy,yaw in levels for r in group if r['value']==value and (r['noise_xy_m'],r['noise_yaw_deg'])==(xy,yaw)]]
                   for value in values])
            if component=='xy_only':
                recovered = [r for r in recovery_summary(grouped, config, component) if r['noise_xy_m']>0]
                table(['σxy [m]', 'estimate 30 σxy [m]', 'estimate τy [s]', 'smallest Dy within 10% [m]', 'tau_y_s', 'mean time [s]', 'paired zero [s]'],
                      [[r[k] for k in ('noise_xy_m','estimate_m','estimate_tau_y_s','minimum_distance_m','tau_y_s','travel_time_s_mean','zero_noise_time_s_mean')] for r in recovered])
                lines.extend(['The smallest qualifying distance is selected among explicit Dy values (0.165–0.60 m), excludes the base law, and requires all 20 seeds to succeed; an absent value means none in this grid qualifies.', ''])
            else:
                ratios = yaw_ratios(grouped)
                table(['σyaw [deg]']+[f"Dy={v if v is not None else 'base'}" for v in values],
                      [[yaw, *[next(r['travel_time_ratio'] for r in ratios if r['noise_yaw_deg']==yaw and r['response_distance_m']==value) for value in values]] for _,yaw in levels if yaw])
                for _, yaw in levels:
                    finite = [r['travel_time_ratio'] for r in ratios if r['noise_yaw_deg']==yaw and r['travel_time_ratio'] is not None]
                    if finite:
                        lines.append(f'At σyaw={yaw:g}°, the noisy/noise-free mean travel-time ratio spans {min(finite):.3f}–{max(finite):.3f} across the base and explicit Dy settings.')
                lines.extend(['', 'Ratios use each setting’s own zero-noise mean; yaw noise is independent of position noise in these runs, and any dependence on Dy is reported rather than assumed absent.', ''])
        heading('Test 5(c): fixed preview on rapid ramp')
        table(['L [m]','method','max eψ [deg]','travel [s]'], [[r['value'],r['method'],r['eval_max_heading_error_deg'],r['travel_time_s']] for r in fifth if r['part']=='c'])
    if historical_verification:
        heading('Verification and provenance')
        lines.extend([historical_verification, ''])
    heading('Follow-up verification and provenance' if historical_verification else 'Verification and provenance')
    timing = {}
    for trial in manifest['trials']:
        r = trial['summary']
        if r['trial_id'] not in timing:
            saved = json.loads((output/'trials'/r['trial_id']/'trial.json').read_text())
            timing[r['trial_id']] = dict(method=r['method'], steps=r['control_steps'], **saved['performance'])
    timing_rows = []
    for method in dict.fromkeys(r['method'] for r in timing.values()):
        group = [r for r in timing.values() if r['method']==method]
        timing_rows.append([method, len(group),
                            sum(r['controller_mean_us']*r['steps'] for r in group)/sum(r['steps'] for r in group),
                            float(np.median([r['controller_p95_us'] for r in group])),
                            max(r['controller_max_us'] for r in group)])
    table(['method','distinct trials','cycle-weighted mean [µs]','median trial p95 [µs]','maximum [µs]'], timing_rows)
    lines.extend(['Computation times measure Python preview, demand and command calculation on the recorded host, excluding noise generation, integration and plotting; they are not a ROS real-time guarantee.', ''])
    attempts = manifest.get('previous_execution_attempts', [])
    elapsed = sum(a.get('simulation_wall_time_s') or 0. for a in attempts) + manifest['simulation_wall_time_s']
    lines.extend([f"Simulation/cache wall time over {len(attempts)+1} execution attempts: {elapsed:.1f} s (current {manifest['simulation_wall_time_s']:.1f} s, {manifest.get('cache_hits',0)} cache hits); source {manifest['source_sha256']}; numerical source {manifest['numerical_source_sha256']}.", '',
                  'The sampled-path tangent fallback and endpoint limits of the compensation identity are specified in `docs/desired_velocity.md`.', ''])
    validation_path = output/'validation.json'
    if validation_path.exists():
        validation = json.loads(validation_path.read_text())
        lines.extend([f"Validation: {validation.get('status')}; {validation.get('conditions')} conditions, {validation.get('distinct_trajectories')} trajectories; runtime {validation.get('wall_time_s'):.1f} s.", ''])
    else:
        lines.extend(['Validation: pending `tools/validate_access_v3.py`.', ''])
    pytest_path = output/'pytest.txt'
    lines.extend(['Pytest: '+(pytest_path.read_text().strip().splitlines()[-1] if pytest_path.exists() else 'pending'), ''])
    changed = subprocess.check_output(['git','status','--short','--untracked-files=all'],cwd=ROOT,text=True).splitlines()
    files = [line[3:] for line in changed if not line[3:].startswith('results/access_v3/')]
    lines.extend(['Changed/new source, configuration, documentation and test files:', '', *[f'- `{f}`' for f in files], '',
                  'Generated files (per-trial NPZ/JSON files are Git-ignored):', '',
                  *[f'- `{p.relative_to(output)}`' for p in sorted(output.rglob('*')) if p.is_file() and 'trials' not in p.relative_to(output).parts and p.name!='REPORT.md'], ''])
    (output/'REPORT.md').write_text('\n'.join(lines))


def outputs(output, rows, manifest):
    grouped = noise_summary(rows)
    config = Config(**manifest['config'])
    recovery = recovery_summary(grouped, config)
    if grouped:
        write_csv(output/'test5'/'noise_summary.csv', grouped)
        write_csv(output/'test5'/'travel_time_recovery.csv', recovery)
        write_csv(output/'test5'/'travel_time_recovery_xy_only.csv', recovery_summary(grouped, config, 'xy_only'))
        write_csv(output/'test5'/'yaw_only_ratios.csv', yaw_ratios(grouped))
    write_csv(output/'performance.csv', [dict(trial_id=t['summary']['trial_id'], **json.loads((output/'trials'/t['summary']['trial_id']/'trial.json').read_text())['performance']) for t in manifest['trials']])
    manifest['v2_parity'] = parity(output, manifest)
    predictions = unchanged_figures(output, rows, manifest)
    figures(output, rows, manifest, grouped)
    report(output, rows, manifest, grouped, recovery, predictions)
