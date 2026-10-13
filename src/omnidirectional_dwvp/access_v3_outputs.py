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
from .access_v3_metrics import comparison_key, reference_arclength
from .config import Config
from .paths import straight_arc_bounds
from .studies import ROOT, write_csv

RESPONSE_COLORS = {None: '#333333', .055: '#a5509a', .11: '#2568a0', .165: '#23845d', .33: '#bf4145'}
NOISE_COLORS = ('#333333', '#2568a0', '#23845d', '#b87510', '#bf4145')
NOISE_KEYS = ('scenario', 'method', 'parameter', 'value', 'fixed_lookahead_m', 'response_distance_m', 'tau_y_s', 'noise_xy_m', 'noise_yaw_deg')


def tag(value):
    return f'{value:g}'.replace('.', 'p')


def label(response, speed):
    if response is None:
        return r"base ($\tau_y=L'/v_{\mathrm{des}}$)"
    return rf'$\tau_y={response/speed:g}$ s'


def load(output, row):
    with np.load(output/'trials'/row['trial_id']/'trajectory.npz') as saved:
        return {k: saved[k] for k in saved}


def finish(fig, ax, stem, xlabel, ylabel, legend=True, legend_fontsize=7, legend_loc='best'):
    ax.set(xlabel=xlabel, ylabel=ylabel)
    ax.grid(alpha=.2, lw=.4)
    if legend:
        ax.legend(frameon=False, fontsize=legend_fontsize, loc=legend_loc)
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
                and (component!='yaw_only' or parameter=='response_distance')]
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
        fig, (ax, base) = plt.subplots(1, 2, figsize=(3.3, 2.3), sharey=True, gridspec_kw={'width_ratios': [4.5, 1]})
    else:
        fig, ax = plt.subplots(figsize=(3.3, 2.3))
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
        base.set(xticks=[], xlabel=label(None, config.translation_speed), xlim=(-.8,len(levels)-.2))
        base.grid(axis='y', alpha=.2, lw=.4)
        ax.set_xlim(0, 1.05*max(r['tau_y_s'] for r in group))
        ax.legend(frameon=False, fontsize=7, loc='upper right')
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
            fig, ax = plt.subplots(figsize=(2.3, 1.65))
            for r in chosen:
                if r['fixed_lookahead_m'] != look:
                    continue
                a = load(output, r)
                d = r['response_distance_m']
                ax.plot(a['times'], a['signed_lateral_errors'], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', lw=1)
            ax.axhline(0, color='.5', lw=.5)
            finish(fig, ax, output/'test1'/f'test1_response_L{tag(look)}_lateral', 'Time [s]', 'Signed lateral error [m]', False)
        fig = plt.figure(figsize=(7.0, .7))
        handles = [Line2D([], [], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', label=label(d, speed))
                   for d in [*manifest['settings']['response_distances'], None]]
        # Retain the standalone legend's original 315 pt exported width.
        fig.legend(handles=handles, loc='center', ncol=3, frameon=False, columnspacing=6.3125)
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
    for group, stem, metric, ylabel in ((ramps, 'orientation', 'eval_max_heading_error_deg', r'$e_{\theta,\max}$ [deg]'),
                                       (arcs, 'incut', 'entry_incut_m', r'$e_{\mathrm{in}}$ [m]')):
        if not group:
            continue
        fig, ax = plt.subplots(figsize=(1.75, 1.55))
        for d in dict.fromkeys(r['response_distance_m'] for r in group):
            series = sorted([r for r in group if r['response_distance_m']==d], key=lambda r:r['fixed_lookahead_m'])
            x = [r['fixed_lookahead_m'] for r in series]
            ax.plot(x, [r[metric] for r in series], color=RESPONSE_COLORS[d], marker='o', ms=3, label=label(d, speed), ls='--' if d is None else '-')
        finish(fig, ax, output/'test2'/f'test2_{stem}_vs_lookahead', r'$L$ [m]', ylabel, legend=False)
    # Response panels (ECPP style): the orientation error along the section and the lateral error along the arc.
    ramp_series = sorted([r for r in ramps if r['response_distance_m'] is not None and abs(r['tau_y_s']-.5)<1e-9
                          and r['fixed_lookahead_m'] in (.055, .165, .33)], key=lambda r: r['fixed_lookahead_m'])
    if ramp_series:
        fig, ax = plt.subplots(figsize=(1.75, 1.55))
        start, length = config.transition_start if hasattr(config, 'transition_start') else 1.0, ramp_series[0]['transition_length_m']
        ax.axvspan(start, start+length, color='#bf4145', alpha=.12, lw=0)
        for r, color in zip(ramp_series, ('#1b9e77', '#d95f02', '#7570b3')):
            a = load(output, r)
            yaw = np.degrees(a['signed_yaw_errors']) if np.max(np.abs(a['signed_yaw_errors'])) < 3.2 else a['signed_yaw_errors']
            ax.plot(a['travel_distance'], yaw, color=color, lw=1, label=rf"$L={r['fixed_lookahead_m']:g}$ m")
        ax.axhline(0, color='.5', lw=.5)
        ax.set_xlim(0.7, 2.3)
        finish(fig, ax, output/'test2'/'test2_orientation_response', 'Travel distance [m]', r'$e_\theta$ [deg]', legend_fontsize=6.5, legend_loc='upper right')
    arc_series = sorted([r for r in arcs if r['fixed_lookahead_m']==.33], key=lambda r: (r['response_distance_m'] is None, r['response_distance_m'] or 0))
    if arc_series:
        fig, ax = plt.subplots(figsize=(1.75, 1.55))
        settings = manifest['settings']
        entry, exit = straight_arc_bounds(settings['arc_straight_in'], settings['arc_radius'], settings['arc_angle_deg'])
        for r in arc_series:
            a = load(output, r)
            d = r['response_distance_m']
            ax.plot(reference_arclength(a), a['signed_lateral_errors'], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', lw=1,
                    label=label(d, speed))
        ax.axhline(0, color='.5', lw=.5)
        for boundary in (entry, exit):
            ax.axvline(boundary, color='.5', ls=':', lw=.7)
        ax.set_xlim(entry - .5, exit + settings['arc_straight_out'] - .5)
        finish(fig, ax, output/'test2'/'test2_arc_lateral_error', 'Arclength [m]', r'$e$ [m]', legend=False)
    if ramps or arcs:
        # Shared legend of the time constants for the Test 2 panels (the lookahead legend stays inside panel (a)).
        fig = plt.figure(figsize=(4.6, .3))
        handles = [Line2D([], [], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', marker='o', ms=3, label=label(d, speed))
                   for d in [.11, .165, .33, None]]
        fig.legend(handles=handles, loc='center', ncol=4, frameon=False, borderaxespad=0)
        save(fig, output/'test2'/'test2_legend')
    if arcs:
        series = [r for r in arcs if r['fixed_lookahead_m']==.33]
        fig, ax = plt.subplots(figsize=(4.5, 3.1))
        inset = ax.inset_axes([.08, .60, .45, .30])
        for i, r in enumerate(series):
            a = load(output, r)
            d = r['response_distance_m']
            # Stack the longer base label to retain the arc panel's natural width.
            arc_label = ('base\n'+r"($\tau_y=\frac{L'}{v_{\mathrm{des}}}$)"
                         if d is None else label(d, speed))
            if i == 0:
                for axis in (ax, inset):
                    axis.plot(a['path'][:, 0], a['path'][:, 1], ':', color='.2', label='Reference')
            for axis in (ax, inset):
                axis.plot(a['poses'][:, 0], a['poses'][:, 1], color=RESPONSE_COLORS[d], ls='--' if d is None else '-', lw=1, label=arc_label)
        entry = manifest['settings']['arc_straight_in']
        inset.set(xlim=(entry-.25, entry+.65), ylim=(-.03, .28))
        inset.tick_params(labelsize=5.5)
        inset.grid(alpha=.2)
        ax.set_aspect('equal')
        ax.indicate_inset_zoom(inset, edgecolor='.5', lw=.6)
        ax.legend(loc='center left', bbox_to_anchor=(1.05, .5), frameon=False, fontsize=7)
        finish(fig, ax, output/'test2'/'test2_arc_paths', 'x [m]', 'y [m]', False)
    for component in ('combined', 'xy_only', 'yaw_only'):
        noise_travel_figure(output, grouped, config, 'response_distance', component)
    noise_travel_figure(output, grouped, config, 'fixed_lookahead')
    noise_travel_figure(output, grouped, config, 'fixed_lookahead', 'xy_only')
    rapid = [r for r in grouped if r['scenario']=='rapid']
    if rapid:
        fig, ax = plt.subplots(figsize=(4.1, 2.6))
        for method, parameter in [('vp', 'adaptive'), ('vp_scaled', 'adaptive'), ('dwvp', 'adaptive'), ('dwvp', 'response_distance')]:
            series = sorted([r for r in rapid if r['method']==method and r['parameter']==parameter], key=lambda r:r['noise_xy_m'])
            ax.errorbar([r['noise_xy_m'] for r in series], [r['eval_max_heading_error_deg_mean'] for r in series],
                        yerr=[r['eval_max_heading_error_deg_std'] or 0 for r in series], lw=1, capsize=2,
                        color='#23845d' if parameter=='response_distance' else COLORS[method],
                        ls=':' if parameter=='response_distance' else STYLES[method], marker='o', ms=3,
                        label='DWVP L=0.11 m, '+label(series[0]['response_distance_m'], speed)
                        if parameter=='response_distance' else LABELS[method])
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


def arc_table(output, rows):
    """Booktabs fragment for the Test 2(b) preview/response comparison."""
    arcs = [r for r in rows if r['test']=='test2' and r['part']=='b']
    if not arcs:
        return
    looks = sorted({r['fixed_lookahead_m'] for r in arcs})
    responses = sorted({r['response_distance_m'] for r in arcs}, key=lambda d: (d is None, d or 0.))
    n = len(looks)
    headings = [rf'${"L=" if i == 0 else ""}{look:g}$'+('~m' if i == n-1 else '')
                for i, look in enumerate(looks)]
    lines = [r'\begin{tabular}{l'+'r'*(2*n)+'}', r'\toprule',
             r'$\tau_y$ [s] & '+rf'\multicolumn{{{n}}}{{c}}{{$e_{{\mathrm{{in}}}}$ [m]}} & '
             +rf'\multicolumn{{{n}}}{{c}}{{$\hat{{\tau}}_{{y,\mathrm{{out}}}}$ [s]}}'+r' \\',
             ' & '+' & '.join(headings*2)+r' \\', r'\midrule']
    for response in responses:
        group = sorted([r for r in arcs if r['response_distance_m']==response], key=lambda r:r['fixed_lookahead_m'])
        title = r"Base ($L'/v_{\mathrm{des}}$)" if response is None else f"{group[0]['tau_y_s']:g}"
        values = ['--' if r[key] is None else f'{r[key]:.{digits}f}'
                  for key, digits in [('entry_incut_m', 3), ('exit_tau_fit_s', 2)] for r in group]
        lines.append(' & '.join([title, *values])+r' \\')
    lines.extend([r'\bottomrule', r'\end{tabular}'])
    (output/'test2'/'test2_arc_table.tex').write_text('\n'.join(lines)+'\n')


def report(output, rows, manifest, grouped, recovery, predictions):
    config = Config(**manifest['config'])
    lines = ['# access_v3 results', '',
             f"{len(rows)} conditions; {len({r['trial_id'] for r in rows})} distinct trajectories; "
             f"{sum(r['success'] for r in rows)} successes, {sum(r['status']=='timeout' for r in rows)} timeouts, "
             f"{sum(r['status'] not in ('success','timeout') for r in rows)} errors/collisions.", '',
             'All errors cover start through settled goal arrival or timeout. Failed travel times and 2% settling times are undefined; noise recovery requires all 20 seeds to succeed. Complete per-condition metrics remain in each test’s `summary.csv`.', '',
             'Signed lateral error uses the closest segment’s left normal. For straight–arc–straight paths, entry_incut_m is the maximum signed error over projected arclength entry ≤ s ≤ exit; exit_bulge_m is the minimum for s > exit. Positive error is inside the CCW arc; negative error after exit is outside. Arclength is reconstructed from reference_segment and reference_fraction on the sampled polyline, not robot travel distance.', '',
             'entry_reaction_m is entry minus the projected arclength of the first saved sample before entry with signed error > 0.001 m, or zero if none exists. exit_tau_fit_s is −1/slope from a least-squares fit of log|e| versus time, using exit ≤ s < polyline length − approach_distance (0.6 m here) and 5–90% of the error magnitude at the first saved sample at/after exit. Fits with at most five samples, zero reference error or nonnegative slope are undefined; neither metric interpolates threshold crossings.', '']

    def heading(title):
        lines.extend(['## '+title, ''])

    def table(headers, data):
        cell = lambda v: fmt(v).replace('|', r'\|')
        lines.extend(['| '+' | '.join(cell(h) for h in headers)+' |', '| '+' | '.join(['---']*len(headers))+' |'])
        lines.extend('| '+' | '.join(cell(v) for v in row)+' |' for row in data)
        lines.append('')

    failures = [r for r in rows if not r['success']]
    if failures:
        from collections import Counter
        counts = Counter((r['test'], r['scenario'], r['method'], r['parameter'], r['value'], r['noise_xy_m'], r['noise_yaw_deg'], r['status']) for r in failures)
        table(['test', 'scene', 'method', 'parameter', 'value', 'σxy [m]', 'σyaw [deg]', 'status', 'count'],
              [[*key, count] for key, count in counts.items()])
    first = [r for r in rows if r['test']=='test1']
    if first:
        heading('Test 1: lateral response')
        lines.extend(['DWVP results below; clipped VP and DWPP comparisons and all full-motion metrics are in [test1/summary.csv](test1/summary.csv). τ fit uses 0.02 < |e| < 0.15 m and t < 20 s. The stopping-distance comparison v²/(2ay) is 0.110 m; it is not a closed-loop guarantee.', ''])
        table(['L [m]', 'Dy [m]', 'τy [s]', 'τ fit [s]', '2% [s]', 'cross [m]', 'travel [s]', 'goal'],
              [[r['fixed_lookahead_m'], 'base' if r['response_distance_m'] is None else r['response_distance_m'],
                r['tau_y_s'], r['tau_fit_s'], r['settling_2pct_time_s'], fmt(r['crossing_m'], 6), r['travel_time_s'], r['success']]
               for r in first if r['method']=='dwvp'])
        for d in manifest['settings']['response_distances']:
            group = [r for r in first if r['method']=='dwvp' and r['response_distance_m']==d]
            settling = [r['settling_2pct_time_s'] for r in group if r['settling_2pct_time_s'] is not None]
            if settling:
                lines.append(f'At Dy={d:g} m, settling across sampled L spans {min(settling):.3f}–{max(settling):.3f} s.')
        lines.extend(['', 'Base τy is the small-error value L/v; DWPP fits are descriptive. Fixed-Dy independence of L is an observation on this finite grid.', ''])
    second = [r for r in rows if r['test']=='test2']
    if second:
        heading('Test 2(a): orientation preview')
        table(['L [m]', 'Dy [m]', 'τy [s]', 'eθ,max [deg]', 'travel [s]', 'goal'],
              [[r['fixed_lookahead_m'], 'base' if r['response_distance_m'] is None else r['response_distance_m'],
                r['tau_y_s'], r['eval_max_heading_error_deg'], r['travel_time_s'], r['success']]
               for r in second if r['part']=='a'])
        heading('Test 2(b): entry into a CCW arc')
        settings = manifest['settings']
        entry, exit = straight_arc_bounds(settings['arc_straight_in'], settings['arc_radius'], settings['arc_angle_deg'])
        lines.extend([f"The reference starts at (0, 0) with zero pose error: a +x straight of {entry:g} m, a CCW {settings['arc_angle_deg']:g}° arc of radius {settings['arc_radius']:g} m, and an outgoing tangent straight of {settings['arc_straight_out']:g} m. There are no transition curves. Tangent heading is continuous; the analytic arclength spacing is {settings['path_spacing']:g} m with the endpoint included.", '',
                      f'Entry and exit arclengths are {entry:g} m and {exit:.9f} m. The entry and exit extrema use the projected-arclength intervals defined above, over the full run. max |e| and eθ,max also cover the full run. Base uses Dy=L′; its tabulated τy=L/v is a small-error label.', ''])
        table(['L [m]', 'Dy [m]', 'τy [s]', 'entry_reaction_m', 'exit_tau_fit_s', 'entry_incut_m', 'exit_bulge_m', 'max |e| [m]', 'eθ,max [deg]', 'travel [s]', 'goal'],
              [[r['fixed_lookahead_m'], 'L′ (base)' if r['response_distance_m'] is None else r['response_distance_m'], r['tau_y_s'],
                fmt(r['entry_reaction_m'], 6), fmt(r['exit_tau_fit_s'], 6),
                fmt(r['entry_incut_m'], 6), f"{r['exit_bulge_m']:.6g}" if r['exit_bulge_m'] is not None else None,
                fmt(r['max_abs_signed_lateral_error_m'], 6),
                r['eval_max_heading_error_deg'], r['travel_time_s'], r['success']]
               for r in second if r['part']=='b'])
        lines.extend(['LaTeX tabular fragment: [test2/test2_arc_table.tex](test2/test2_arc_table.tex).', ''])
        arcs = [r for r in second if r['part']=='b']
        reaction_spread = max(max(g)-min(g) for look in {r['fixed_lookahead_m'] for r in arcs}
                              for g in [[r['entry_reaction_m'] for r in arcs if r['fixed_lookahead_m']==look]])
        lines.extend([f'At fixed L, the sampled 1 mm reaction distance varies by up to {reaction_spread:.6f} m across response settings; this threshold metric does not establish exact independence from τy.', ''])
        lines.extend(['Updated figures (PDF and PNG): `test2/test2_arc_lateral_error`, `test2/test2_incut_vs_lookahead`, and `test2/test2_arc_paths` (entry inset). The lateral-error panel uses L=0.33 m and projected arclength from 0.5 m before entry to 0.5 m before the endpoint; dotted vertical lines mark entry and exit. Orientation panels and the shared `test2_legend` retain their definitions and styling.', ''])
    for test, title in (('test3', 'Test 3: constraints'), ('test4', 'Test 4: obstacle speed regulation')):
        group = [r for r in rows if r['test']==test]
        if not group:
            continue
        heading(title)
        proof = manifest['v2_parity']
        lines.extend([f"All common trajectory arrays and metrics match v2: maximum absolute difference {proof['maximum_absolute_difference']:g} across {proof['compared_conditions']} Test 3/4 conditions. Full results: [{test}/summary.csv]({test}/summary.csv).", ''])
        if test=='test3':
            table(['method', 'conditions', 'successes', 'largest eθ,max [deg]', 'largest command violation [%]'],
                  [[method, len(g), sum(r['success'] for r in g), max(r['eval_max_heading_error_deg'] for r in g),
                    max(r['command_constraint_violation_pct'] for r in g)]
                   for method in dict.fromkeys(r['method'] for r in group) for g in [[r for r in group if r['method']==method]]])
            if predictions:
                lines.extend(['The unchanged overshoot comparison is in [test3/overshoot_prediction.csv](test3/overshoot_prediction.csv).', ''])
        else:
            table(['regulation', 'method', 'near speed [m/s]', 'max ep [m]', 'eθ,max [deg]', 'travel [s]', 'command violation [%]'],
                  [[r['value'], r['method'], r['mean_near_obstacle_speed_m_s'], r['eval_max_position_error_m'],
                    r['eval_max_heading_error_deg'], r['travel_time_s'], r['command_constraint_violation_pct']] for r in group])
    fifth = [r for r in rows if r['test']=='test5']
    if fifth:
        heading('Test 5: matched travel time and noise')
        selected = [r for r in fifth if r['parameter']=='match_reference' or r['trial_id'] in {m['best_trial_id'] for m in manifest['time_matches'].values()}]
        table(['method', 'scale', 'travel [s]', 'eθ,max [deg]', 'max ep [m]'],
              [[r['method'], r['value'], r['travel_time_s'], r['eval_max_heading_error_deg'], r['eval_max_position_error_m']] for r in selected])
        lines.extend(['The unchanged v2 travel-time search scales x/y velocity limits, translation demand and nominal speed together; acceleration limits remain fixed. All noise means, sample SDs, counts and failures are retained in [test5/noise_summary.csv](test5/noise_summary.csv). Zero-noise seeds repeat deterministic trajectories.', '',
                      'Recovery below means all 20 seeds succeed and mean travel time is at most 1.1 times the mean at the same noise-free setting. Empty values mean no sampled setting qualifies. The comparison estimate is 30 σxy [m], not a guarantee.', ''])
        table(['noise component', 'parameter', 'σxy [m]', 'σyaw [deg]', 'estimate [m]', 'smallest within 10% [m]', 'travel [s]', 'paired zero [s]'],
              [[component, r['parameter'], r['noise_xy_m'], r['noise_yaw_deg'], r['estimate_m'], r['minimum_distance_m'],
                r['travel_time_s_mean'], r['zero_noise_time_s_mean']]
               for component, recovered in [('combined', recovery), ('xy_only', recovery_summary(grouped, config, 'xy_only'))]
               for r in recovered if r['noise_xy_m']>0])
        ratios = yaw_ratios(grouped)
        if ratios:
            table(['σyaw [deg]', 'min noisy/zero travel-time ratio', 'max ratio'],
                  [[yaw, min(v), max(v)] for yaw in sorted({r['noise_yaw_deg'] for r in ratios})
                   for v in [[r['travel_time_ratio'] for r in ratios if r['noise_yaw_deg']==yaw and r['travel_time_ratio'] is not None]] if v])
        lines.extend(['Detailed recovery and yaw-only ratios: [combined](test5/travel_time_recovery.csv), [position-only](test5/travel_time_recovery_xy_only.csv), [yaw-only](test5/yaw_only_ratios.csv). Test 5(c) rapid-ramp fixed-preview results remain in [test5/summary.csv](test5/summary.csv) and `test5/test5_orientation_vs_lookahead.{pdf,png}`.', ''])
    heading('Verification and provenance')
    timing = {}
    for trial in manifest['trials']:
        r = trial['summary']
        if r['trial_id'] not in timing:
            saved = json.loads((output/'trials'/r['trial_id']/'trial.json').read_text())
            timing[r['trial_id']] = dict(method=r['method'], steps=r['control_steps'], **saved['performance'])
    table(['method', 'distinct trials', 'cycle-weighted mean [µs]', 'median trial p95 [µs]', 'maximum [µs]'],
          [[method, len(g), sum(r['controller_mean_us']*r['steps'] for r in g)/sum(r['steps'] for r in g),
            float(np.median([r['controller_p95_us'] for r in g])), max(r['controller_max_us'] for r in g)]
           for method in dict.fromkeys(r['method'] for r in timing.values())
           for g in [[r for r in timing.values() if r['method']==method]]])
    lines.extend(['Times measure Python preview, demand and command calculation, excluding noise generation, integration and plotting; they are not a ROS real-time guarantee.', '',
                  f"Current simulation/cache wall time: {manifest['simulation_wall_time_s']:.1f} s; {manifest.get('cache_hits', 0)} cache hits; {manifest['workers']} workers. Earlier attempts remain in the manifest.", ''])
    if manifest.get('total_wall_time_s') is not None:
        lines.extend([f"Current run including figure regeneration: {manifest['total_wall_time_s']:.1f} s.", ''])
    attempts = manifest.get('previous_execution_attempts', [])
    if attempts:
        previous = attempts[-1]
        lines.extend([f"Immediately preceding run: simulation/cache {fmt(previous.get('simulation_wall_time_s'), 1)} s; including figures {fmt(previous.get('total_wall_time_s'), 1)} s. Complete timing history is in the manifest.", ''])
    validation_path = output/'validation.json'
    validation = json.loads(validation_path.read_text()) if validation_path.exists() else {}
    if validation.get('source_sha256') == manifest['source_sha256']:
        lines.extend([f"Validation: {validation['status']}; {validation['conditions']} conditions, {validation['distinct_trajectories']} trajectories; runtime {validation['wall_time_s']:.1f} s; {validation['protected_v2_files_unchanged']} protected files unchanged.", ''])
    else:
        lines.extend(['Validation: pending `tools/validate_access_v3.py` for this source revision.', ''])
    pytest_path = output/'pytest.txt'
    lines.extend(['Pytest: '+(pytest_path.read_text().strip().splitlines()[-1] if pytest_path.exists() else 'pending'), '',
                  f"Source SHA-256: `{manifest['source_sha256']}`; numerical source: `{manifest['numerical_source_sha256']}`.", '',
                  f"All {len(manifest.get('figures', []))} PDF/PNG figure pairs are listed in [manifest.json](manifest.json). Per-trial histories under `trials/` are Git-ignored. The compensation identity and endpoint limits are in `docs/desired_velocity.md`.", ''])
    changed = subprocess.check_output(['git', 'status', '--short', '--untracked-files=all'], cwd=ROOT, text=True).splitlines()
    files = [line[3:] for line in changed if not line[3:].startswith('results/access_v3/')]
    lines.extend(['Changed source, configuration, documentation and tests: '+', '.join(f'`{f}`' for f in files)+'.', ''])
    assert len(lines) <= 240, f'Report exceeds 240 lines: {len(lines)}'
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
    arc_table(output, rows)
    predictions = unchanged_figures(output, rows, manifest)
    figures(output, rows, manifest, grouped)
    manifest['figures'] = [str(p.relative_to(output).with_suffix('')) for p in sorted(output.glob('test*/*.pdf'))]
    report(output, rows, manifest, grouped, recovery, predictions)
