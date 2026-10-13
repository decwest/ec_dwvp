"""Validate saved access_v3 evidence without changing any v2 artifact."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from omnidirectional_dwvp.access_studies import atomic_json, numerical_hash
from omnidirectional_dwvp.access_v3 import plan_conditions, selected_studies
from omnidirectional_dwvp.access_v3_metrics import extra_metrics
from omnidirectional_dwvp.access_v3_outputs import noise_summary, recovery_summary, yaw_ratios, parity, report
from omnidirectional_dwvp.access_metrics import evaluate
from omnidirectional_dwvp.config import Config
from omnidirectional_dwvp.controller import compute_command, preview_target, desired_vector
from omnidirectional_dwvp.geometry import Obstacle
from omnidirectional_dwvp.metrics import project_reference, signed_lateral_errors
from omnidirectional_dwvp.simulation import Result
from omnidirectional_dwvp.studies import ROOT, code_hash, sha
from validate_access_v2 import validate_goal_and_settling


def csv_matches(path, expected):
    with path.open() as stream:
        reader=csv.DictReader(stream)
        actual=list(reader)
        assert reader.fieldnames==list(dict.fromkeys(k for row in expected for k in row)),path
    assert len(actual)==len(expected),path
    for actual_row,expected_row in zip(actual,expected):
        assert all(v==('' if expected_row.get(k) is None else str(expected_row[k])) for k,v in actual_row.items()),path


def validate(root,study='all'):
    start=perf_counter()
    assert root.is_dir() and not root.is_symlink()
    manifest_path=root/('manifest.json' if study=='all' else f'manifest_{study}.json')
    manifest=json.loads(manifest_path.read_text())
    assert manifest['complete'] and manifest['suite']=='access_v3'
    assert manifest['planner_version']=='access_v3.3'
    assert manifest['source_sha256']==code_hash()
    assert manifest['numerical_source_sha256']==numerical_hash()
    assert manifest['dependency_lock_sha256']==hashlib.sha256((ROOT/'uv.lock').read_bytes()).hexdigest()
    assert manifest['config_file_sha256']==hashlib.sha256((ROOT/'configs/access_v3.yaml').read_bytes()).hexdigest()
    cfg=Config(**manifest['config'])
    selected=selected_studies(study)
    expected=plan_conditions(cfg,manifest['settings'],manifest['seed'],selected)
    assert Counter(sha(c) for c in expected)==Counter(t['condition_id'] for t in manifest['trials'] if not (t['condition']['test']=='test5' and t['condition']['part']=='a'))
    assert Counter(sha(c) for c in manifest['planned_conditions'])==Counter(t['condition_id'] for t in manifest['trials'])
    assert len({t['condition_id'] for t in manifest['trials']})==len(manifest['trials'])
    rows=[t['summary'] for t in manifest['trials']]
    for test in selected:
        csv_matches(root/test/'summary.csv',[r for r in rows if r['test']==test])
    unique={}
    replays=0
    for trial in manifest['trials']:
        row,spec=trial['summary'],trial['spec']
        config=Config(**spec['config'])
        if row['test'] in ('test1','test2') or (row['test']=='test5' and row['part']=='b'):
            distance=config.response_distance if config.response_distance is not None else config.fixed_lookahead
            assert row['tau_y_s']==(distance/config.translation_speed if distance is not None else None)
        identity=row['trial_id']
        assert sha(spec)==identity
        assert sha(trial['condition'])==row['condition_id']
        if identity in unique:
            continue
        unique[identity]=True
        directory=root/'trials'/identity
        metadata=json.loads((directory/'trial.json').read_text())
        assert metadata['trial_id']==identity and metadata['spec']==spec
        assert row['status']!='error',metadata
        assert all(row[k]==v for k,v in metadata['metrics'].items())
        with np.load(directory/'trajectory.npz') as saved:
            a={k:saved[k] for k in saved}
        config=Config(**spec['config'])
        n=len(a['applied'])
        assert len(a['poses'])==len(a['times'])==n+1
        assert hashlib.sha256(a['path'].tobytes()).hexdigest()==spec['path_sha256']
        assert np.isfinite(a['poses']).all() and np.isfinite(a['demands']).all()
        for name in ('commands','applied','measured','demands','response_rules'):
            assert len(a[name])==n
        np.testing.assert_array_equal(a['commands'],a['applied'])
        np.testing.assert_array_equal(a['applied'],a['measured'])
        np.testing.assert_array_equal(a['times'],np.arange(n+1)*config.dt)
        assert np.all(a['commands']>=a['boxes'][:,0]-1e-10)
        assert np.all(a['commands']<=a['boxes'][:,1]+1e-10)
        previous=np.vstack((np.zeros(3),a['applied'][:-1]))
        assert np.all(abs(a['commands']-previous)<=config.acceleration*config.dt+1e-10)
        assert np.all(a['commands']>=config.lower-1e-10) and np.all(a['commands']<=config.upper+1e-10)
        ref,errors,yaw,segments,fractions=project_reference(a['poses'],a['path'])
        for name,expected_array in [('reference_poses',ref),('position_errors',errors),('yaw_errors',yaw),
                                    ('reference_segment',segments),('reference_fraction',fractions),
                                    ('signed_lateral_errors',signed_lateral_errors(a['poses'],a['path'],ref,segments))]:
            np.testing.assert_array_equal(a[name],expected_array)
        validate_goal_and_settling(a,row,spec)
        assert sum(row[f'response_{r}_steps'] for r in ('inactive','chord','tangent','goal'))==n
        for code,rule in enumerate(('inactive','chord','tangent','goal')):
            assert row[f'response_{rule}_steps']==int((a['response_rules']==code).sum())
        if config.response_distance is None or spec['method'] in ('dwpp','rpp'):
            assert np.all(a['response_rules']==0)
        result=Result(a,metadata['metrics'],metadata['performance'])
        recomputed=evaluate(result,config,spec['scenario'],spec['initial_pose'],None,spec['ramp_start'])
        recomputed.update(extra_metrics(result,config,spec['scenario']))
        assert all(row[k]==v for k,v in recomputed.items()),identity
        # Reconstruct the exact seeded pose observations and replay representative
        # command cycles, including rule switches and the final terminal cycle.
        noise=np.random.default_rng(spec['seed']).normal(size=(n,3))*np.array([config.noise_xy,config.noise_xy,config.noise_yaw])
        switches=np.flatnonzero(np.r_[False,np.diff(a['response_rules'])!=0])
        indices=np.unique(np.r_[np.linspace(0,n-1,5,dtype=int),switches[:5]])
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(a['path'][:,:2],axis=0),axis=1))]
        for i in indices:
            out=compute_command(a['poses'][i]+noise[i],previous[i],a['path'],arc,spec['method'],config,tuple(Obstacle(*o) for o in spec['obstacles']))
            np.testing.assert_array_equal(out.command,a['commands'][i])
            np.testing.assert_array_equal(out.desired,a['demands'][i])
            assert {None:0,'chord':1,'tangent':2,'goal':3}[out.response_rule]==a['response_rules'][i]
            replays+=1
        if len(unique)%250==0:
            print(f'Validated {len(unique)} distinct trajectories',flush=True)
    grouped=noise_summary(rows)
    position_only_base=[r for r in rows if r['test']=='test5' and r['part']=='b'
                        and r['parameter']=='fixed_lookahead' and r['noise_xy_m']>0 and r['noise_yaw_deg']==0]
    if 'test5' in selected:
        settings=manifest['settings']
        assert Counter((r['fixed_lookahead_m'],r['noise_xy_m'],r['seed']) for r in position_only_base)==Counter(
            (look,xy,seed) for look in settings['fixed_lookaheads'] for xy in settings['noise_xy_only_levels']
            for seed in range(manifest['seed'],manifest['seed']+settings['noise_seeds']))
        assert all(r['scenario']=='offset' and r['method']=='dwvp' and r['response_distance_m'] is None for r in position_only_base)
    recovery=recovery_summary(grouped,cfg)
    if grouped:
        csv_matches(root/'test5/noise_summary.csv',grouped)
        csv_matches(root/'test5/travel_time_recovery.csv',recovery)
        csv_matches(root/'test5/travel_time_recovery_xy_only.csv',recovery_summary(grouped,cfg,'xy_only'))
        csv_matches(root/'test5/yaw_only_ratios.csv',yaw_ratios(grouped))
        assert all(r['n']==manifest['settings']['noise_seeds'] for r in grouped)
        for group in grouped:
            seeds=[r['seed'] for r in rows if r['test']=='test5' and r['part']=='b' and
                   all(r.get(k)==group[k] for k in ('scenario','method','parameter','value','noise_xy_m','noise_yaw_deg'))]
            assert sorted(seeds)==list(range(manifest['seed'],manifest['seed']+manifest['settings']['noise_seeds']))
    for method,match in manifest.get('time_matches',{}).items():
        best=next(r for r in rows if r['trial_id']==match['best_trial_id'])
        assert best['method']==method and best['value']==match['best_scale']
        assert match['matched'] and abs(best['travel_time_s']-match['target_time_s'])<=manifest['settings']['match_tolerance_s']+1e-10
    proof=parity(root,manifest)
    if study=='all':
        assert proof['compared_conditions']==136 and proof['maximum_absolute_difference']==0
    expected_stems=[]
    if 'test1' in selected:
        expected_stems += ['test1/test1_response_L'+v+'_lateral' for v in ('0p11','0p165','0p33')]
        expected_stems += ['test1/test1_'+s for s in ('response_legend','settling_vs_response','crossing_vs_response')]
    if 'test2' in selected:
        expected_stems += ['test2/test2_'+s for s in ('orientation_response','orientation_vs_lookahead',
                                                    'incut_vs_lookahead','arc_lateral_error','arc_paths','legend')]
    if 'test3' in selected:
        expected_stems += ['test3/test3_acceleration_ratio_ramp_'+v+'_'+s for v in ('1','0p6','0p4','0p3','0p2') for s in ('overshoot','heading_integral','heading')]
        expected_stems += ['test3/test3_ramp_0p3_acceleration_'+v+'_'+s for v in ('0p25','1') for s in ('speed','yaw','signed_heading')]
        expected_stems += ['test3/test3_'+s for s in ('acceleration_ratio_legend','acceleration_time_series_legend','max_orientation_vs_length')]
    if 'test4' in selected:
        expected_stems += ['test4/test4_'+s for s in ('scene','cost0_approach1_distance_speed','cost1_approach1_distance_speed')]
    if 'test5' in selected:
        expected_stems += ['test5/test5_'+s for s in ('noise_travel_time_vs_response','noise_travel_time_vs_response_xy_only','noise_travel_time_vs_response_yaw_only','noise_travel_time_vs_lookahead','noise_travel_time_vs_lookahead_xy_only','noise_orientation','orientation_vs_lookahead')]
    figure_stems=[str(p.relative_to(root).with_suffix('')) for p in sorted(root.glob('test*/*.pdf'))]
    assert manifest['figures']==figure_stems
    assert set(expected_stems)<=set(figure_stems)
    assert set(figure_stems)=={str(p.relative_to(root).with_suffix('')) for p in root.glob('test*/*.png')}
    from matplotlib.image import imread
    for stem in figure_stems:
        pdf=(root/(stem+'.pdf')).read_bytes()
        assert pdf.startswith(b'%PDF') and len(pdf)>1000
        pixels=imread(root/(stem+'.png'))
        assert pixels.shape[0]>50 and float(pixels.std())>.01
    fixtures=json.loads((ROOT/'fixtures/desired_vector_cases.json').read_text())['cases']
    assert len(fixtures)>=60
    for case in fixtures:
        config=Config(fixed_lookahead=case['lookahead_distance'],response_distance=case['response_distance'],
                      vp_translation_speed=case['translation_speed'],orientation_time_weight=case['orientation_time_weight'],
                      min_orientation_time=case['min_orientation_time'])
        pose,path=np.array(case['pose']),np.array(case['path'])
        target,_,_,goal=preview_target(pose,np.zeros(3),path,config)
        np.testing.assert_array_equal(target,case['target'])
        demand,rule=desired_vector(pose,target,config,path,goal_region=goal,return_rule=True)
        np.testing.assert_array_equal(demand,case['expected_desired_vector'])
        assert rule==case['response_rule']
    preserved=root/'v2_protected_sha256.json'
    if not preserved.exists():
        preserved=ROOT/'build/preserved_v2_sha256.json'
    protected=0
    if preserved.exists():
        hashes=json.loads(preserved.read_text())
        for path,digest in hashes.items():
            assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest,path
        protected=len(hashes)
    validation=dict(status='passed',conditions=len(rows),distinct_trajectories=len(unique),
                    command_cycles_replayed=replays,noise_groups=len(grouped),
                    position_only_base_conditions=len(position_only_base),
                    position_only_base_recovery=[r for r in recovery_summary(grouped,cfg,'xy_only')
                                                 if r['parameter']=='fixed_lookahead' and r['noise_xy_m']>0],
                    desired_vector_fixture_cases=len(fixtures),
                    required_figure_pairs=len(expected_stems),figure_pairs=len(list(root.glob('test*/*.pdf'))),
                    v2_parity=proof,protected_v2_files_unchanged=protected,
                    source_sha256=code_hash(),numerical_source_sha256=numerical_hash(),
                    wall_time_s=perf_counter()-start)
    atomic_json(root/('validation.json' if study=='all' else f'validation_{study}.json'),validation)
    predictions=[]
    if (root/'test3/overshoot_prediction.csv').exists():
        with (root/'test3/overshoot_prediction.csv').open() as stream:
            for r in csv.DictReader(stream):
                for k in ('transition_length_m','value','predicted_heading_overshoot_deg','observed_heading_overshoot_deg','prediction_error_deg'):
                    r[k]=float(r[k]) if r[k] else None
                predictions.append(r)
    report(root,rows,manifest,grouped,recovery,predictions)
    assert len((root/'REPORT.md').read_text().splitlines()) <= 240
    print(json.dumps(validation,indent=2))
    return validation


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--study',default='all',choices=('all','test1','test2','test3','test4','test5'))
    args=parser.parse_args()
    validate(args.output,args.study)
