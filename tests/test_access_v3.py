"""Response law, portable fixtures and versioned study coverage."""
from dataclasses import replace
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
from omnidirectional_dwvp.paths import straight_path, arc_path
from omnidirectional_dwvp.simulation import simulate
from omnidirectional_dwvp.access_v3 import plan_conditions, selected_studies
from omnidirectional_dwvp.access_v3_metrics import fitted_time_constant
from omnidirectional_dwvp.access_v3_outputs import recovery_summary

ROOT = Path(__file__).resolve().parents[1]


def profile():
    raw = yaml.safe_load((ROOT/'configs/access_v3.yaml').read_text())
    settings = raw.pop('study')
    return Config(**raw), settings


@pytest.mark.parametrize('value', [0,-.1,float('nan'),float('inf')])
def test_response_distance_validation(value):
    with pytest.raises(ValueError):
        Config(response_distance=value)


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
    assert counts==dict(test1=30,test2=30,test3=132,test4=4,test5=2424)
    assert not any(c['method']=='vp_scaled_accel' for c in rows)
    for c in rows:
        if c['test'] in ('test3','test4'):
            assert c['config']['response_distance'] is None
    noise=[c for c in rows if c['test']=='test5' and c['part']=='b']
    groups={}
    for c in noise:
        key=json.dumps({k:v for k,v in c.items() if k!='seed'},sort_keys=True)
        groups.setdefault(key,[]).append(c['seed'])
    assert len(groups)==120
    assert all(v==list(range(20)) for v in groups.values())


def test_arc_path_limits_and_endpoint():
    a=arc_path(1,3,.005)
    assert len(a)==601
    np.testing.assert_allclose(a[0],[0,0,0])
    np.testing.assert_allclose(a[-1],[math.sin(3),1-math.cos(3),3])
    for values in ((0,1,.01),(1,7,.01),(1,1,0)):
        with pytest.raises(ValueError):
            arc_path(*values)


def test_noise_recovery_requires_all_seeds():
    base=dict(scenario='offset',parameter='response_distance',noise_yaw_deg=0.,n=20,success_count=20,travel_time_s_n=20)
    rows=[dict(base,value=d,noise_xy_m=xy,travel_time_s_mean=time) for d,xy,time in [( .165,0,20),(.33,0,22),(.165,.01,21),(.33,.01,23)]]
    rows[2].update(success_count=19,travel_time_s_n=19)
    recovered=recovery_summary(rows,Config())
    assert next(r for r in recovered if r['noise_xy_m']==.01)['minimum_distance_m']==.33


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
