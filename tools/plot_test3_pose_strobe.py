"""Draw two Test 3 pose strobes from saved trajectories, without simulation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from omnidirectional_dwvp.access_plotting import COLORS, HEADING_METHODS, LABELS, style
from omnidirectional_dwvp.access_studies import atomic_json, numerical_hash
from omnidirectional_dwvp.studies import ROOT
import matplotlib.pyplot as plt


def sample_poses(times, poses):
    """Sample global 0.5 s ticks, from x > 0.5 until the first x > 2.5.

    Interpolation uses saved time/pose pairs (with unwrapped yaw). The current
    30 Hz histories already contain every requested tick. Include x == 2.5,
    exclude the first tick beyond it, and never resume after leaving the panel.
    """
    ticks = np.arange(np.ceil(times[0] / .5), np.floor(times[-1] / .5) + 1) * .5
    sampled = np.column_stack([np.interp(ticks, times, values)
                               for values in (poses[:, 0], np.unwrap(poses[:, 2]))])
    first = np.flatnonzero(sampled[:, 0] > .5)[0]
    exits = np.flatnonzero(sampled[first:, 0] > 2.5)
    stop = first + exits[0] if len(exits) else len(ticks)
    return ticks[first:stop], sampled[first:stop]


def pose_figure(samples):
    """Use lane heights only for layout; horizontal position and yaw are data."""
    fig, ax = plt.subplots(figsize=(3.3, 2.0))
    fig.subplots_adjust(left=.25, right=.97, bottom=.22, top=.94)
    ax.set(xlim=(.5, 2.5), ylim=(-.15, 1.05), xlabel=r'$x$ [m]', yticks=[])
    ax.set_aspect('equal')
    ax.set_xticks(np.arange(.5, 2.51, .5))
    ax.spines['left'].set_visible(False)
    ax.axvspan(1., 1.3, color='#bf4145', alpha=.12, lw=0, zorder=-1)
    for lane, name in zip((.9, .6, .3, 0.), ('Path', *(LABELS[m] for m in HEADING_METHODS))):
        ax.axhline(lane, color='.8', lw=.45, zorder=0)
        ax.text(-.04, lane, name, transform=ax.get_yaxis_transform(),
                ha='right', va='center', fontsize=7)

    def arrows(xs, lane, yaw, length, color):
        ax.quiver(xs, np.full_like(xs, lane), length*np.cos(yaw), length*np.sin(yaw),
                  angles='xy', scale_units='xy', scale=1, pivot='tail', color=color, width=.0025,
                  headwidth=3, headlength=4, headaxislength=3.5, zorder=3)

    reference_x = np.arange(.5, 2.51, .1)
    reference_yaw = np.clip((reference_x - 1.) / .3, 0., 1.) * np.pi/2
    arrows(reference_x, .9, reference_yaw, .12, '#bf4145')
    for lane, method in zip((.6, .3, 0.), HEADING_METHODS):
        xs, yaw = samples[method][:, 0], samples[method][:, 1]
        arrows(xs, lane, yaw, .12, COLORS[method])
    return fig


def generate(output):
    assert output.is_dir() and not output.is_symlink()
    assert (output/'test3').is_dir() and not (output/'test3').is_symlink()
    output = output.resolve()
    assert output.is_relative_to(ROOT)
    manifest = json.loads((output/'manifest.json').read_text())
    assert manifest['complete'] and manifest['suite'] == 'access_v3'
    assert manifest['numerical_source_sha256'] == numerical_hash()
    rows = [t['summary'] for t in manifest['trials']]
    details = {}
    with plt.rc_context():
        style()
        for scale, rho in ((1., '1p33'), (.25, '5p33')):
            stem = f'test3/test3_pose_strobe_rho{rho}'
            samples, counts = {}, {}
            for method in HEADING_METHODS:
                selected = [r for r in rows if r['test'] == 'test3' and r['part'] == 'acceleration'
                            and r['parameter'] == 'acceleration_scale' and r['value'] == scale
                            and r['transition_length_m'] == .3 and r['method'] == method]
                if len(selected) != 1:
                    raise ValueError(f'Expected one Test 3 trajectory for {method}, scale={scale}')
                row = selected[0]
                with np.load(output/'trials'/row['trial_id']/'trajectory.npz') as saved:
                    ticks, samples[method] = sample_poses(saved['times'], saved['poses'])
                counts[method] = dict(trial_id=row['trial_id'], first_time_s=float(ticks[0]),
                                      last_time_s=float(ticks[-1]), count=len(ticks))
            fig = pose_figure(samples)
            try:
                # Avoid tight bounding-box cropping: exported dimensions are 3.3 x 2.0 in.
                for suffix in ('.pdf', '.png'):
                    fig.savefig((output/stem).with_suffix(suffix), dpi=240, bbox_inches=None)
            finally:
                plt.close(fig)
            details[stem] = dict(acceleration_scale=scale, methods=counts)
    manifest['figures'] = [str(p.relative_to(output).with_suffix(''))
                           for p in sorted(output.glob('test*/*.pdf'))]
    manifest['test3_pose_strobe'] = dict(
        simulation_runs=0, interval_s=.5, transition_length_m=.3,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), figures=details)
    atomic_json(output/'manifest.json', manifest)
    return manifest['test3_pose_strobe']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    print(json.dumps(generate(parser.parse_args().output), indent=2))
