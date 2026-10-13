"""Response law, portable fixtures and versioned study coverage."""
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from omnidirectional_dwvp.config import Config
from omnidirectional_dwvp.controller import desired_vector, preview_target, compute_command, response_direction
from omnidirectional_dwvp.geometry import wrap
from omnidirectional_dwvp.metrics import project_reference, signed_lateral_errors
from omnidirectional_dwvp.paths import straight_path, arc_path, straight_arc_path, straight_arc_bounds
from omnidirectional_dwvp.simulation import Result, simulate
from omnidirectional_dwvp.access_v3 import plan_conditions, selected_studies
from omnidirectional_dwvp.access_v3_metrics import fitted_time_constant, extra_metrics, reference_arclength
from omnidirectional_dwvp.access_v3_outputs import noise_summary, noise_travel_figure, recovery_summary, yaw_ratios

ROOT = Path(__file__).resolve().parents[1]


def profile():
    raw = yaml.safe_load((ROOT/'configs/access_v3.yaml').read_text())
    settings = raw.pop('study')
    return Config(**raw), settings


@pytest.mark.parametrize('value', [0,-.1,float('nan'),float('inf')])
def test_response_distance_validation(value):
    with pytest.raises(ValueError):
        Config(response_distance=value)


@pytest.mark.parametrize('value', [0, -.1, float('nan'), float('inf')])
def test_response_time_validation(value):
    with pytest.raises(ValueError, match='response time'):
        Config(response_time=value)


def test_response_time_normalization_and_serialization():
    cfg = Config(response_time=1.5, vp_translation_speed=.22, fixed_lookahead=.11)
    assert cfg.response_distance == .33
    assert Config(**asdict(cfg)) == cfg
    assert replace(cfg, noise_xy=.01).response_distance == .33
    assert Config(response_time=1.5, vp_translation_speed=.1).response_distance == pytest.approx(.15)
    assert Config(response_time=.5).response_distance == .11
    with pytest.raises(ValueError, match='only one'):
        Config(response_distance=.33, response_time=1.5)
    with pytest.raises(ValueError, match='positive translation speed'):
        Config(response_time=1.5, vp_translation_speed=0.)
    path = straight_path(1.)
    a = simulate(path, 'dwvp', cfg, initial_pose=[0,.2,0])
    b = simulate(path, 'dwvp', Config(response_distance=.33, vp_translation_speed=.22, fixed_lookahead=.11), initial_pose=[0,.2,0])
    for key in a.arrays:
        np.testing.assert_array_equal(a.arrays[key], b.arrays[key])


def old_desired_vector(pose,target,config):
    delta = target[:2] - pose[:2]
    c,s = math.cos(pose[2]),math.sin(pose[2])
    body = np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
    distance = float(np.linalg.norm(body))
    speed = config.translation_speed
    translation = np.zeros(2) if distance <= 1e-6 else speed*body/distance
    time = config.min_orientation_time
    if speed > 1e-12:
        time = max(time,config.orientation_time_weight*distance/speed)
    return np.r_[translation,wrap(target[2]-pose[2])/time]


def test_none_bit_exact_on_random_paths_and_poses():
    rng = np.random.default_rng(728)
    for _ in range(400):
        path = np.cumsum(rng.normal(size=(8,3)),axis=0)
        pose = rng.normal(size=3)
        cfg = Config(fixed_lookahead=float(rng.uniform(.01,1)),vp_translation_speed=float(rng.uniform(0,.5)))
        target,_,_,goal = preview_target(pose,rng.normal(size=3),path,cfg)
        np.testing.assert_array_equal(desired_vector(pose,target,cfg,path,goal_region=goal),old_desired_vector(pose,target,cfg))
    for delta in (0,1e-8,1e-6,1e-5):
        pose=np.array([.1,.2,.3]);target=pose+np.array([delta,0,.1])
        np.testing.assert_array_equal(desired_vector(pose,target,Config()),old_desired_vector(pose,target,Config()))


@pytest.mark.parametrize('response', [.165,.33])
def test_requested_time_constant_and_goal(response):
    cfg,_ = profile()
    cfg = replace(cfg,fixed_lookahead=.11,response_distance=response)
    result = simulate(straight_path(),'dwvp',cfg,initial_pose=[0,.5,0])
    a = result.arrays
    tau = fitted_time_constant(a['times'],a['signed_lateral_errors'])
    assert result.metrics['success']
    assert abs(tau/(response/.22)-1) < .05
    assert sum(result.metrics[f'response_{r}_steps'] for r in ('chord','tangent','goal','inactive'))==result.metrics['steps']
    assert result.metrics['response_goal_steps'] > 0


def test_compensation_identity_on_random_interior_projections():
    rng = np.random.default_rng(28)
    for _ in range(500):
        theta=rng.uniform(-math.pi,math.pi)
        tangent=np.array([math.cos(theta),math.sin(theta)])
        normal=np.array([-tangent[1],tangent[0]])
        projection=rng.normal(size=2)
        error=rng.uniform(-.5,.5)
        pose=projection+error*normal
        chord=rng.normal(size=2)
        length=np.linalg.norm(chord)
        response=rng.uniform(.01,1.)
        closed=chord/length-error/response*normal
        compensation=(projection+chord-pose)/length+(1/length-1/response)*error*normal
        np.testing.assert_allclose(closed/np.linalg.norm(closed),compensation/np.linalg.norm(compensation),rtol=0,atol=1e-12)


def test_fixture_roundtrip():
    data=json.loads((ROOT/'fixtures/desired_vector_cases.json').read_text())
    assert len(data['cases'])>=60
    assert {c['response_rule'] for c in data['cases']}=={None,'chord','tangent','goal'}
    for case in data['cases']:
        cfg=Config(fixed_lookahead=case['lookahead_distance'],response_distance=case['response_distance'],
                   vp_translation_speed=case['translation_speed'],orientation_time_weight=case['orientation_time_weight'],
                   min_orientation_time=case['min_orientation_time'])
        pose,path=np.array(case['pose']),np.array(case['path'])
        target,_,nearest,goal=preview_target(pose,np.zeros(3),path,cfg)
        np.testing.assert_array_equal(target,case['target'])
        assert bool(goal)==case['goal_region'] and nearest==case['nearest_pose']
        desired,rule=desired_vector(pose,target,cfg,path,goal_region=goal,return_rule=True)
        np.testing.assert_array_equal(desired,case['expected_desired_vector'])
        assert rule==case['response_rule']


def test_segment_projection_sign_and_sampled_nearest_fallback():
    path=straight_path(1,.1)
    pose=np.array([.023,.5,0.])
    cfg=Config(fixed_lookahead=.11,response_distance=.33)
    target,_,_,goal=preview_target(pose,np.zeros(3),path,cfg)
    direction,rule=response_direction(pose,target,path,.33,goal)
    assert rule=='tangent' and direction[0]>0 and direction[1]<0
    np.testing.assert_allclose(direction,np.array([1,-.5/.33])/np.hypot(1,.5/.33))
    arc=arc_path()
    segment=100
    midpoint=(arc[segment,:2]+arc[segment+1,:2])/2
    tangent=arc[segment+1,:2]-arc[segment,:2];tangent/=np.linalg.norm(tangent)
    normal=np.array([-tangent[1],tangent[0]])
    poses=np.c_[np.stack([midpoint+.01*normal,midpoint-.01*normal]),[0.,0.]]
    reference,_,_,indices,_=project_reference(poses,arc)
    np.testing.assert_allclose(signed_lateral_errors(poses,arc,reference,indices),[.01,-.01],atol=1e-14)


def test_goal_cap_and_endpoint_fallback():
    path=straight_path(1,.1)
    cfg=Config(response_distance=.33,fixed_lookahead=.11)
    for pose in (np.array([.96,.02,.4]),np.array([1.02,.03,.4])):
        target,_,_,goal=preview_target(pose,np.zeros(3),path,cfg)
        assert goal
        new,rule=desired_vector(pose,target,cfg,path,goal_region=goal,return_rule=True)
        np.testing.assert_allclose(new,old_desired_vector(pose,target,replace(cfg,response_distance=None)),atol=1e-14)
        assert rule=='goal'


@pytest.mark.parametrize('method',['dwpp','rpp'])
def test_differential_references_ignore_response_distance(method):
    path=straight_path(1,.01);pose=np.array([.2,.1,.4]);cfg=Config()
    a=compute_command(pose,np.zeros(3),path,path[:,0],method,cfg)
    b=compute_command(pose,np.zeros(3),path,path[:,0],method,replace(cfg,response_distance=.33))
    np.testing.assert_array_equal(a.command,b.command)
    np.testing.assert_array_equal(a.desired,b.desired)
    assert b.response_rule is None


def test_v3_grid_and_noise_seeds():
    cfg,settings=profile()
    rows=plan_conditions(cfg,settings,0,selected_studies('all'))
    counts={test:sum(c['test']==test for c in rows) for test in selected_studies('all')}
    assert counts==dict(test1=30,test2=36,test3=132,test4=4,test5=5624)
    orientation = [c for c in rows if c['test']=='test2' and c['part']=='a']
    assert len(orientation)==24
    assert {(c['config']['fixed_lookahead'], c['config']['response_distance']) for c in orientation}=={
        (look, response) for look in (.055,.08,.11,.165,.22,.33) for response in (.11,.165,.33,None)}
    added = [c for c in orientation if c['config']['response_distance']==.165]
    assert len(added)==6 and all(c['config']['response_distance']/cfg.translation_speed==.75 for c in added)
    assert not any(c['method']=='vp_scaled_accel' for c in rows)
    for c in rows:
        if c['test'] in ('test3','test4'):
            assert c['config']['response_distance'] is None
    noise=[c for c in rows if c['test']=='test5' and c['part']=='b']
    groups={}
    for c in noise:
        key=json.dumps({k:v for k,v in c.items() if k!='seed'},sort_keys=True)
        groups.setdefault(key,[]).append(c['seed'])
    assert len(groups)==280
    assert all(v==list(range(20)) for v in groups.values())
    decomposition=[c for c in noise if bool(c['config']['noise_xy']) != bool(c['config']['noise_yaw'])]
    assert len(decomposition)==2880
    response_rows=[c for c in decomposition if c['parameter']=='response_distance']
    assert len(response_rows)==960
    assert all(c['method']=='dwvp' and c['config']['fixed_lookahead']==.11 for c in response_rows)
    for xy,yaw in ([(v,0.) for v in (.002,.005,.01,.02)] + [(0.,math.radians(v)) for v in (.1,.2,.5,1.)]):
        subset=[c for c in response_rows if (c['config']['noise_xy'],c['config']['noise_yaw'])==(xy,yaw)]
        assert len(subset)==120
        assert {c['config']['response_distance'] for c in subset}=={None,.165,.22,.33,.44,.60}
    base_rows=[c for c in decomposition if c['parameter']=='fixed_lookahead']
    assert len(base_rows)==640
    assert all(c['method']=='dwvp' and c['scenario']['name']=='offset'
               and c['config']['response_distance'] is None and c['config']['noise_yaw']==0 for c in base_rows)
    for xy in (.002,.005,.01,.02):
        subset=[c for c in base_rows if c['config']['noise_xy']==xy]
        assert len(subset)==160
        assert {c['config']['fixed_lookahead'] for c in subset}=={.055,.08,.11,.165,.22,.33,.44,.60}


def test_fixed_response_lookahead_grid_includes_paired_zero_noise():
    from collections import Counter
    cfg, settings = profile()
    rows = [c for c in plan_conditions(cfg, settings, 7, ('test5',))
            if c['parameter']=='fixed_lookahead_at_response']
    assert len(rows)==1600
    assert Counter((c['value'], c['config']['response_distance'], c['config']['noise_xy'], c['seed'])
                   for c in rows)==Counter((look, response, xy, seed)
                                          for look in (.055,.08,.11,.165,.22,.33,.44,.60)
                                          for response in (.165,.33) for xy in (0.,.002,.005,.01,.02)
                                          for seed in range(7,27))
    assert all(c['test']=='test5' and c['part']=='b' and c['method']=='dwvp'
               and c['scenario']==dict(name='offset',kind='offset',offset=.5)
               and c['config']['fixed_lookahead']==c['value'] and c['config']['noise_yaw']==0 for c in rows)
    assert {c['config']['response_distance']/cfg.translation_speed for c in rows}=={.75,1.5}


def test_noise_summary_separates_responses_at_same_lookahead():
    base = dict(test='test5', part='b', scenario='offset', method='dwvp',
                parameter='fixed_lookahead_at_response', value=.11, fixed_lookahead_m=.11,
                noise_xy_m=.002, noise_yaw_deg=0., success=True, status='success')
    rows = [dict(base, response_distance_m=response, tau_y_s=response/.22, seed=seed,
                 travel_time_s=time, eval_position_error_integral_m_s=integral)
            for response, seed, time, integral in ((.165,0,20.,.4),(.165,1,22.,.6),(.33,0,30.,.8),(.33,1,34.,1.))]
    grouped = noise_summary(rows)
    assert [r['n'] for r in grouped]==[2,2]
    assert [r['travel_time_s_mean'] for r in grouped]==[21.,32.]
    np.testing.assert_allclose([r['travel_time_s_std'] for r in grouped], [2**.5, 8**.5])
    np.testing.assert_allclose([r['eval_position_error_integral_m_s_mean'] for r in grouped], [.5,.9])


@pytest.mark.parametrize('component,parameter', [('xy_only','response_distance'), ('xy_only','fixed_lookahead'),
                                               ('combined','response_distance'), ('combined','fixed_lookahead'),
                                               ('yaw_only','response_distance')])
def test_noise_figure_axes_and_series(tmp_path, monkeypatch, component, parameter):
    import matplotlib.pyplot as plt
    from omnidirectional_dwvp import access_v3_outputs as outputs
    cfg, _ = profile()
    grouped = []
    levels = [(0.,0.),(.002,0.),(.005,0.),(.01,0.),(.02,0.),(.002,.1),(0.,.1)]
    for xy,yaw in levels:
        for kind, values, responses in [('response_distance', [None,.165,.33], [None]),
                                       ('fixed_lookahead', [.055,.11,.6], [None]),
                                       ('fixed_lookahead_at_response', [.055,.11,.6], [.165,.33])]:
            for value in values:
                for response in responses:
                    distance = value if kind=='response_distance' else response
                    grouped.append(dict(scenario='offset', parameter=kind, value=value, response_distance_m=distance,
                                        tau_y_s=(distance or .11)/cfg.translation_speed, noise_xy_m=xy, noise_yaw_deg=yaw,
                                        travel_time_s_mean=20+100*xy+(distance or 0), travel_time_s_std=.1,
                                        eval_position_error_integral_m_s_mean=.5+xy+(distance or 0)))
    saved = []
    monkeypatch.setattr(outputs, 'save', lambda fig, stem: saved.append((fig, stem)))
    noise_travel_figure(tmp_path, grouped, cfg, parameter, component)
    fig, stem = saved[0]
    try:
        np.testing.assert_allclose(fig.get_size_inches(), [3.3,2.3])
        response = parameter=='response_distance'
        assert stem.name=='test5_noise_travel_time_vs_'+('response' if response else 'lookahead')+('' if component=='combined' else '_'+component)
        ax = fig.axes[0]
        assert ax.get_xlabel()==(r'$\tau_y$ [s]' if response else r'$L$ [m]')
        assert ax.get_ylabel()==r'$T_m$ [s]'
        travel = [line for line in ax.lines if line.get_marker()=='o']
        if component=='xy_only' and response:
            assert len(fig.axes)==3
            right = fig.axes[2]
            assert right.get_ylabel()==r'$I_p$ [m$\cdot$s]'
            assert len(right.lines)==len(travel)==5
            for (xy,_), left, integral in zip(levels[:5], travel, right.lines):
                assert left.get_linestyle()=='-'
                assert integral.get_linestyle()=='--' and integral.get_marker()=='None'
                assert integral.get_color()==left.get_color() and integral.get_linewidth()<left.get_linewidth()
                np.testing.assert_allclose(integral.get_xdata(), [.75,1.5])
                np.testing.assert_allclose(integral.get_ydata(), [.665+xy,.83+xy])
            assert len(ax.get_legend().get_texts())==5
            assert len([line for line in fig.axes[1].lines if line.get_marker()=='o'])==5
        elif component=='xy_only':
            assert len(fig.axes)==1 and len(travel)==8
            for xy, base, fixed in zip((.002,.005,.01,.02), travel[::2], travel[1::2]):
                assert base.get_linestyle()=='--' and fixed.get_linestyle()=='-'
                assert base.get_color()==fixed.get_color()
                np.testing.assert_allclose(fixed.get_ydata(orig=False), [20+100*xy+.165]*3)
                np.testing.assert_allclose(fixed.get_xdata(orig=False), [.055,.11,.6])
            labels = [t.get_text() for t in ax.get_legend().get_texts()]
            assert len(labels)==6
            assert labels[-2:]==[r'EC-DWVP ($\tau_y=0.75$ s)', r"base ($\tau_y=L'/v_{\mathrm{des}}$)"]
        else:
            assert len(fig.axes)==(2 if response else 1) and len(travel)==2
            assert all(line.get_linestyle()=='-' for line in travel)
        dotted = [line for line in ax.lines if line.get_linestyle()==':']
        noisy_xy = [.002,.005,.01,.02] if component=='xy_only' else [.002] if component=='combined' else []
        np.testing.assert_allclose([line.get_xdata()[0] for line in dotted],
                                   [outputs.noise_lookahead_estimate(cfg, xy)/(cfg.translation_speed if response else 1) for xy in noisy_xy])
    finally:
        plt.close(fig)


def test_response_panels_symbols_legends_and_arc_inset(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    from omnidirectional_dwvp import access_v3_outputs as outputs
    cfg, settings = profile()
    rows = []
    for c in plan_conditions(cfg, settings, 0, ('test1', 'test2')):
        if c['method'] != 'dwvp':
            continue
        look, response = c['config']['fixed_lookahead'], c['config']['response_distance']
        rows.append(dict(test=c['test'], part=c['part'], method='dwvp', fixed_lookahead_m=look,
                         response_distance_m=response, tau_y_s=(response or look)/cfg.translation_speed,
                         transition_length_m=.3, eval_max_heading_error_deg=look*100+(response or 0),
                         entry_incut_m=.02, settling_2pct_time_s=2., crossing_m=0.))
    arrays = synthetic_arc_result([1.8, 2.2, 3., 4.], [.01,.02,.03,.001]).arrays
    arrays.update(signed_yaw_errors=np.array([0.,.1,.2,0.]), travel_distance=np.array([.7,1.,1.3,2.3]))
    monkeypatch.setattr(outputs, 'load', lambda output, row: arrays)
    captured, bounds = {}, {}

    def capture(fig, stem, **kwargs):
        captured[stem.name] = fig
        bounds[stem.name] = kwargs
        plt.close(fig)

    monkeypatch.setattr(outputs, 'save', capture)
    outputs.figures(tmp_path, rows, dict(config=asdict(cfg), settings=settings), [])
    for look in ('0p11', '0p165', '0p33'):
        fig = captured[f'test1_response_L{look}_lateral']
        ax = fig.axes[0]
        np.testing.assert_allclose(fig.get_size_inches(), [2.3,1.65])
        assert (ax.get_xlabel(), ax.get_ylabel()) == (r'$t$ [s]', r'$e$ [m]')
    for name, columns in (('test1_response_legend',5), ('test2_legend',4)):
        fig = captured[name]
        assert fig.legends[0]._ncols == len(fig.legends[0].get_texts()) == columns
    np.testing.assert_allclose(captured['test1_response_legend'].get_size_inches(), [7.,.3])
    assert bounds['test1_response_legend'] == dict(bbox_inches=None)
    orientation = captured['test2_orientation_vs_lookahead'].axes[0]
    assert len(orientation.lines) == 4
    added = orientation.lines[1]
    assert added.get_color() == outputs.RESPONSE_COLORS[.165]
    np.testing.assert_allclose(added.get_xdata(), [.055,.08,.11,.165,.22,.33])
    np.testing.assert_allclose(added.get_ydata(), np.array([.055,.08,.11,.165,.22,.33])*100+.165)
    response = captured['test2_orientation_response'].axes[0]
    assert (response.get_xlabel(), response.get_ylabel()) == (r'$x$ [m]', r'$e_\theta$ [deg]')
    assert [line.get_label() for line in response.lines[:3]] == [r'$L=0.055$ m',r'$L=0.165$ m',r'$L=0.33$ m']
    for line in response.lines[:3]:
        np.testing.assert_allclose(line.get_xdata(), arrays['travel_distance'])
        np.testing.assert_allclose(line.get_ydata(), np.degrees(arrays['signed_yaw_errors']))
    lateral = captured['test2_arc_lateral_error'].axes[0]
    assert (lateral.get_xlabel(), lateral.get_ylabel()) == (r'$s$ [m]', r'$e$ [m]')
    np.testing.assert_allclose(lateral.lines[0].get_xdata(), reference_arclength(arrays))
    fig = captured['test2_arc_paths']
    np.testing.assert_allclose(fig.get_size_inches(), [2.3,2.])
    assert bounds['test2_arc_paths'] == dict(bbox_inches=None)
    ax = fig.axes[0]
    assert (ax.get_xlabel(), ax.get_ylabel()) == (r'$x$ [m]', r'$y$ [m]')
    assert ax.get_xlim() == ax.get_ylim() == (-.2,3.2) and ax.get_aspect() == 1
    inset, = ax.child_axes
    assert inset.get_aspect() == 1
    np.testing.assert_allclose(inset.get_xlim(), [1.75,2.65])
    np.testing.assert_allclose(inset.get_ylim(), [-.03,.28])
    for axis in (ax, inset):
        assert axis.get_legend() is None and axis.get_title() == ''
        assert len(axis.lines) == 5
        reference, *curves = axis.lines
        assert reference.get_color() == '.2' and reference.get_linestyle() == ':'
        np.testing.assert_allclose(reference.get_xydata(), arrays['path'][:,:2])
        assert [line.get_color() for line in curves] == [outputs.RESPONSE_COLORS[d] for d in (.11,.165,.33,None)]
        assert [line.get_linestyle() for line in curves] == ['-','-','-','--']


def test_arc_path_limits_and_endpoint():
    a=arc_path(1,3,.005)
    assert len(a)==601
    np.testing.assert_allclose(a[0],[0,0,0])
    np.testing.assert_allclose(a[-1],[math.sin(3),1-math.cos(3),3])
    for values in ((0,1,.01),(1,7,.01),(1,1,0)):
        with pytest.raises(ValueError):
            arc_path(*values)


def test_straight_arc_endpoint_and_spacing():
    path = straight_arc_path()
    entry, exit = straight_arc_bounds()
    assert (entry, exit) == (2., 2. + math.pi/2)
    np.testing.assert_allclose(path[0], [0., 0., 0.])
    np.testing.assert_allclose(path[-1], [3., 3., math.pi/2])
    # Recover analytic arclength from the coordinates and tangent headings.
    s = np.where(path[:, 2] == 0., path[:, 0],
                 np.where(path[:, 2] < math.pi/2, entry + path[:, 2], exit + path[:, 1] - 1.))
    np.testing.assert_allclose(np.diff(s)[:-1], .005, atol=1e-14)
    assert 0 < s[-1]-s[-2] <= .005
    assert np.all(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 0)
    # Non-default geometry also ends along the outgoing tangent.
    path = straight_arc_path(1., 2., 60., 3., .007)
    angle = math.pi/3
    np.testing.assert_allclose(path[-1], [1+2*math.sin(angle)+3*math.cos(angle),
                                        2*(1-math.cos(angle))+3*math.sin(angle), angle])


def test_straight_arc_join_tangents():
    # Choose a spacing putting both joins on the global sample grid.
    spacing = math.pi/2000
    path = straight_arc_path(400*spacing, 1., 90., 400*spacing, spacing)
    segment_heading = np.arctan2(*np.diff(path[:, :2], axis=0)[:, ::-1].T)
    for index, heading in ((400, 0.), (1400, math.pi/2)):
        assert path[index, 2] == pytest.approx(heading, abs=1e-14)
        np.testing.assert_allclose(segment_heading[index-1:index+1], heading, atol=spacing/2+1e-12)
    np.testing.assert_allclose(path[400, :2], [400*spacing, 0.])
    np.testing.assert_allclose(path[1400, :2], [400*spacing+1, 1.])


@pytest.mark.parametrize('parameter,value', [('straight_in', 0), ('radius', -1),
    ('angle_deg', 361), ('angle_deg', float('nan')), ('straight_out', float('inf')), ('spacing', 0)])
def test_straight_arc_invalid_geometry(parameter, value):
    with pytest.raises(ValueError):
        straight_arc_path(**{parameter: value})


def test_straight_arc_metrics_use_projected_arclength():
    path = straight_arc_path()
    segments = np.array([100, 450, 520, 600, 730, 850, 950])
    offsets = np.array([.5, .04, -.2, .12, -.07, .3, -.02])
    delta = path[segments+1, :2] - path[segments, :2]
    tangent = delta / np.linalg.norm(delta, axis=1)[:, None]
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    xy = (path[segments, :2]+path[segments+1, :2])/2 + offsets[:, None]*normal
    poses = np.c_[xy, np.zeros(len(xy))]
    ref, _, _, indices, fractions = project_reference(poses, path)
    arrays = dict(path=path, poses=poses, reference_segment=indices, reference_fraction=fractions,
                  signed_lateral_errors=signed_lateral_errors(poses, path, ref, indices),
                  times=np.arange(len(poses), dtype=float),
                  response_rules=np.zeros(len(poses)-1, dtype=int),
                  travel_distance=np.arange(len(poses))*10.)
    scene = dict(kind='straight_arc', straight_in=2., radius=1., angle_deg=90., straight_out=2.)
    metrics = extra_metrics(Result(arrays, {}, {}), Config(), scene)
    assert metrics['entry_incut_m'] == pytest.approx(.12)
    assert metrics['exit_bulge_m'] == pytest.approx(-.07)
    assert metrics['max_abs_signed_lateral_error_m'] == pytest.approx(.5)
    assert metrics['incut_prediction_m'] is metrics['incut_relative_deviation'] is None
    assert metrics['steady_lateral_error_m'] is None
    lengths = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    expected_s = np.cumsum(np.r_[0., lengths])[segments] + lengths[segments]/2
    np.testing.assert_allclose(reference_arclength(arrays), expected_s, atol=1e-14)
    # A run ending before entry has no arc or exit observations.
    for key in ('reference_segment', 'reference_fraction', 'signed_lateral_errors', 'times'):
        arrays[key] = arrays[key][:1]
    arrays['response_rules'] = np.array([], dtype=int)
    metrics = extra_metrics(Result(arrays, {}, {}), Config(), scene)
    assert metrics['entry_incut_m'] is metrics['exit_bulge_m'] is None
    assert metrics['exit_tau_fit_s'] is None


def synthetic_arc_result(arclengths, errors, times=None):
    """Place synthetic poses at signed normal offsets from the sampled path."""
    path = straight_arc_path()
    delta = np.diff(path[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    starts = np.r_[0., np.cumsum(lengths)]
    segments = np.searchsorted(starts, arclengths, side='right')-1
    fractions = (np.asarray(arclengths)-starts[segments])/lengths[segments]
    tangent = delta[segments]/lengths[segments, None]
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    xy = path[segments, :2]+fractions[:, None]*delta[segments]+np.asarray(errors)[:, None]*normal
    poses = np.c_[xy, np.zeros(len(xy))]
    ref, _, _, segments, fractions = project_reference(poses, path)
    arrays = dict(path=path, poses=poses, reference_segment=segments, reference_fraction=fractions,
                  signed_lateral_errors=signed_lateral_errors(poses, path, ref, segments),
                  times=np.arange(len(poses), dtype=float) if times is None else np.asarray(times),
                  response_rules=np.zeros(len(poses)-1, dtype=int))
    return Result(arrays, {}, {})


@pytest.mark.parametrize('tau', [.5, .75, 1.5])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_exit_tau_fit_recovers_exponential_and_excludes_other_intervals(tau, sign):
    scene = dict(kind='straight_arc', straight_in=2., radius=1., angle_deg=90., straight_out=2.)
    _, exit = straight_arc_bounds()
    path = straight_arc_path()
    end = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()
    # All timestamps exceed Test 1's 20 s cutoff. Pre-exit, >90%, <5%, and
    # goal-approach samples deliberately do not follow the fitted exponential.
    fit_times = np.linspace(0., 2*tau, 25)
    s = np.r_[exit-.02, exit+.01, exit+.015, exit+np.linspace(.02, 1.1, 25),
              exit+1.2, end-.59, end-.3, end-.1]
    errors = sign*np.r_[.025, .1, .095, .08*np.exp(-fit_times/tau), .004, .04, .03, .02]
    times = 25+np.r_[0., .1, .2, .4+fit_times, 1+2*tau, 2+2*tau, 3+2*tau, 4+2*tau]
    result = synthetic_arc_result(s, errors, times)
    metrics = extra_metrics(result, Config(approach_distance=.6), scene)
    assert metrics['exit_tau_fit_s'] == pytest.approx(tau, rel=1e-11)
    # Enlarging the excluded goal region leaves too few samples for a fit.
    assert extra_metrics(result, Config(approach_distance=1.98), scene)['exit_tau_fit_s'] is None


@pytest.mark.parametrize('count', [5, 6])
def test_exit_tau_fit_requires_more_than_five_samples(count):
    scene = dict(kind='straight_arc', straight_in=2., radius=1., angle_deg=90., straight_out=2.)
    _, exit = straight_arc_bounds()
    times = np.arange(count+1)*.2
    result = synthetic_arc_result(exit+.01+times*.1, .02*np.exp(-times/.75), times)
    tau = extra_metrics(result, Config(), scene)['exit_tau_fit_s']
    if count == 5:
        assert tau is None
    else:
        assert tau == pytest.approx(.75, rel=1e-11)


@pytest.mark.parametrize('errors', [np.zeros(8), np.r_[.02, np.linspace(.002, .016, 7)]])
def test_exit_tau_fit_undefined_for_zero_reference_or_nondecaying_fit(errors):
    scene = dict(kind='straight_arc', straight_in=2., radius=1., angle_deg=90., straight_out=2.)
    _, exit = straight_arc_bounds()
    result = synthetic_arc_result(exit+np.linspace(.01, .5, len(errors)), errors)
    assert extra_metrics(result, Config(), scene)['exit_tau_fit_s'] is None


@pytest.mark.parametrize('errors,expected', [
    ([-.02, .001, .0011, .004, .01], .1),
    ([-.02, .001, .0005, .0009, .01], 0.),
])
def test_entry_reaction_uses_first_strict_inward_threshold_before_entry(errors, expected):
    scene = dict(kind='straight_arc', straight_in=2., radius=1., angle_deg=90., straight_out=2.)
    result = synthetic_arc_result([1.5, 1.7, 1.9, 1.99, 2.1], errors)
    # Accumulated robot travel is unrelated to the required projected arclength.
    result.arrays['travel_distance'] = np.arange(5)*10.
    metrics = extra_metrics(result, Config(), scene)
    assert metrics['entry_reaction_m'] == pytest.approx(expected, abs=1e-14)
    assert metrics['exit_tau_fit_s'] is None


def test_v3_preview_uses_straight_arc_scene():
    cfg, settings = profile()
    conditions = [c for c in plan_conditions(cfg, settings, 0, ('test2',)) if c['part']=='b']
    assert len(conditions) == 12
    assert all(c['scenario'] == dict(name='straight_arc', kind='straight_arc', straight_in=2.,
                                    radius=1., angle_deg=90., straight_out=2.) for c in conditions)
    assert {(c['config']['fixed_lookahead'], c['config']['response_distance']) for c in conditions} == {
        (look, response) for look in (.11, .165, .33) for response in (.11, .165, .33, None)}


@pytest.mark.parametrize('parameter', ['response_distance', 'fixed_lookahead'])
def test_noise_recovery_requires_all_seeds(parameter):
    base=dict(scenario='offset',parameter=parameter,noise_yaw_deg=0.,n=20,success_count=20,travel_time_s_n=20)
    rows=[dict(base,value=d,noise_xy_m=xy,travel_time_s_mean=time) for d,xy,time in [( .165,0,20),(.33,0,22),(.165,.01,21),(.33,.01,23)]]
    rows[2].update(success_count=19,travel_time_s_n=19)
    recovered=recovery_summary(rows,Config(), 'xy_only')
    assert next(r for r in recovered if r['noise_xy_m']==.01)['minimum_distance_m']==.33


def test_noise_components_use_their_own_paired_baselines():
    base=dict(scenario='offset',parameter='response_distance',n=20,success_count=20,travel_time_s_n=20)
    rows=[dict(base,value=d,tau_y_s=d/.22,noise_xy_m=xy,noise_yaw_deg=yaw,travel_time_s_mean=time)
          for d,xy,yaw,time in [(.165,0,0,20),(.33,0,0,30),(.165,.01,.5,24),(.33,.01,.5,32),
                               (.165,.01,0,21),(.33,.01,0,31),(.165,0,.5,24),(.33,0,.5,33)]]
    assert recovery_summary(rows,Config())[-1]['minimum_distance_m']==.33
    assert recovery_summary(rows,Config(),'xy_only')[-1]['minimum_distance_m']==.165
    assert [r['travel_time_ratio'] for r in yaw_ratios(rows)]==[1.2,1.1]


def test_base_recovery_uses_same_lookahead_baseline_and_keeps_unrecovered_levels():
    base=dict(scenario='offset',noise_yaw_deg=0.,n=20,success_count=20,travel_time_s_n=20)
    rows=[dict(base,parameter='fixed_lookahead',value=look,noise_xy_m=xy,travel_time_s_mean=time)
          for look,xy,time in [(.055,0,20),(.11,0,30),(.055,.01,23),(.11,.01,33),(.055,.02,24),(.11,.02,34)]]
    rows += [dict(base,parameter='response_distance',value=.055,noise_xy_m=xy,travel_time_s_mean=25)
             for xy in (0,.01,.02)]
    recovered=[r for r in recovery_summary(rows,Config(),'xy_only') if r['parameter']=='fixed_lookahead']
    assert recovered[1]['minimum_distance_m']==.11
    assert recovered[1]['zero_noise_time_s_mean']==30
    assert recovered[2]['minimum_distance_m'] is None


def test_chord_length_response_matches_original_on_interior_projections():
    rng=np.random.default_rng(306)
    for _ in range(100):
        heading=rng.uniform(-math.pi,math.pi)
        path=straight_path(4,.02,heading)
        tangent=np.array([math.cos(heading),math.sin(heading)])
        normal=np.array([-tangent[1],tangent[0]])
        pose=np.r_[rng.uniform(.5,3.)*tangent+rng.uniform(-.08,.08)*normal,rng.uniform(-math.pi,math.pi)]
        cfg=Config(fixed_lookahead=.33)
        target,_,_,goal=preview_target(pose,np.zeros(3),path,cfg)
        ref,*_=project_reference(pose[None,:],path)
        active=replace(cfg,response_distance=float(np.linalg.norm(target[:2]-ref[0,:2])))
        new=desired_vector(pose,target,active,path,goal_region=goal)
        old=old_desired_vector(pose,target,cfg)
        np.testing.assert_allclose(new[:2],old[:2],rtol=0,atol=1e-12)
        assert new[2]==old[2]


@pytest.mark.parametrize('method',['vp','vp_scaled','vp_scaled_accel','dwvp'])
def test_omnidirectional_baselines_share_compensated_demand(method):
    cfg=Config(response_distance=.33,fixed_lookahead=.11)
    pose=np.array([.537,-.07,.4]);path=straight_path()
    target,_,_,goal=preview_target(pose,np.zeros(3),path,cfg)
    expected,rule=desired_vector(pose,target,cfg,path,goal_region=goal,return_rule=True)
    output=compute_command(pose,np.zeros(3),path,path[:,0],method,cfg)
    np.testing.assert_array_equal(output.desired,expected)
    assert output.response_rule==rule
