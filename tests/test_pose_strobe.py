"""Generate the Test 3 artifacts from a small saved trajectory fixture."""
import importlib.util
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle
import numpy as np

from ec_dwvp.access_plotting import COLORS, HEADING_METHODS
from ec_dwvp.access_studies import numerical_hash


def test_pose_strobe_generation(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'pose_strobe', Path(__file__).resolve().parents[1]/'tools/plot_test3_pose_strobe.py')
    plotting = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(plotting)
    # tmp_path normally lives under build; keep the fixture independent of cwd.
    monkeypatch.setattr(plotting, 'ROOT', tmp_path)
    output = tmp_path/'results'
    (output/'test3').mkdir(parents=True)
    # x == 0.5 is excluded, x == 2.5 is included, a pause yields overlapping
    # poses, and re-entry after the first exit must not add another marker.
    times = np.arange(0., 3.51, .25)
    x = np.interp(times, np.arange(0., 3.51, .5), [.1, .5, .75, .75, 1.25, 2.5, 2.6, 2.])
    poses = np.column_stack((x, np.full_like(x, 99.), times*.4))
    trials = []
    for scale in (1., .25):
        for method in HEADING_METHODS:
            identity = f'{scale}-{method}'
            trial = output/'trials'/identity
            trial.mkdir(parents=True)
            np.savez(trial/'trajectory.npz', times=times, poses=poses)
            trials.append(dict(summary=dict(test='test3', part='acceleration', method=method,
                                            parameter='acceleration_scale', value=scale,
                                            transition_length_m=.3, trial_id=identity)))
    (output/'manifest.json').write_text(json.dumps(dict(
        complete=True, suite='access_v3', numerical_source_sha256=numerical_hash(), trials=trials)))
    figures = []
    original = plotting.pose_figure

    def capture(samples):
        fig = original(samples)
        figures.append(fig)
        return fig

    monkeypatch.setattr(plotting, 'pose_figure', capture)
    details = plotting.generate(output)
    manifest = json.loads((output/'manifest.json').read_text())
    assert details['simulation_runs'] == 0
    assert manifest['test3_pose_strobe'] == details
    assert manifest['figures'] == ['test3/test3_pose_strobe_rho1p33', 'test3/test3_pose_strobe_rho5p33']
    for stem, fig in zip(manifest['figures'], figures):
        assert (output/(stem+'.pdf')).read_bytes().startswith(b'%PDF')
        pixels = plt.imread(output/(stem+'.png'))
        assert pixels.shape[:2] == (480, 792) and pixels.std() > .01
        np.testing.assert_allclose(fig.get_size_inches(), [3.3, 2.])
        assert len(fig.axes) == 1
        ax = fig.axes[0]
        assert ax.get_xlim() == (.5, 2.5) and ax.get_aspect() == 1
        assert ax.get_title() == ax.get_ylabel() == '' and len(ax.get_yticks()) == 0
        assert [text.get_text() for text in ax.texts] == ['Path', 'Clipped VP', 'Scaled VP', 'EC-DWVP']
        assert all(text.get_fontsize() == 7 for text in ax.texts)
        assert not any(isinstance(patch, Circle) for patch in ax.patches)
        assert ax.get_xlabel() == r'$x$ [m]'
        for i, (method, lane) in enumerate(zip(HEADING_METHODS, (.6, .3, 0.))):
            info = details['figures'][stem]['methods'][method]
            assert (info['first_time_s'], info['last_time_s'], info['count']) == (1., 2.5, 4)
            arrows = ax.collections[i+1]
            np.testing.assert_allclose(arrows.get_offsets(), [[xpos, lane] for xpos in (.75,.75,1.25,2.5)])
            np.testing.assert_allclose(arrows.get_facecolors(), [to_rgba(COLORS[method])])
            assert arrows.pivot == 'tail' and arrows.width <= .003
            yaw = np.array([1., 1.5, 2., 2.5])*.4
            np.testing.assert_allclose(arrows.U, .12*np.cos(yaw))
            np.testing.assert_allclose(arrows.V, .12*np.sin(yaw))
        reference = ax.collections[0]
        reference_x = np.arange(.5, 2.51, .1)
        np.testing.assert_allclose(reference.get_offsets()[:, 0], reference_x)
        np.testing.assert_allclose(np.hypot(reference.U, reference.V), .12)
        np.testing.assert_allclose(np.arctan2(reference.V, reference.U),
                                   np.clip((reference_x-1.)/.3, 0., 1.)*np.pi/2)
