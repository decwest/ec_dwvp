"""Versioned five-test planner and runner for the response-distance study."""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from itertools import repeat
from time import perf_counter
import hashlib
import json
import platform
import sys
import numpy as np
from .access_studies import atomic_json, execute_trial, make_condition, numerical_hash, plan_conditions as plan_v2
from .studies import ROOT, sha, code_hash, git_revision, cpu_model, write_csv


def selected_studies(study):
    if study not in ('all', 'test1', 'test2', 'test3', 'test4', 'test5'):
        raise ValueError('access_v3 supports all and test1 through test5')
    return tuple(f'test{i}' for i in range(1, 6)) if study == 'all' else (study,)


def plan_conditions(config, settings, seed, selected):
    conditions = []
    offset = dict(name='offset', kind='offset', offset=.5)
    rapid = dict(name='rapid', kind='ramp', transition_length=.3)
    methods = ('vp', 'vp_scaled', 'dwvp')
    def add(test, part, scene, cfg, methods=('dwvp',), parameter='', value=None, seeds=None):
        if test not in selected:
            return
        for method in methods:
            for trial_seed in ([seed] if seeds is None else seeds):
                c = make_condition(test, part, scene, method, cfg, trial_seed, settings, parameter, value)
                c['suite'] = 'access_v3'
                conditions.append(c)
    for look in settings['response_lookaheads']:
        for response in [*settings['response_distances'], None]:
            cfg = replace(config, fixed_lookahead=look, response_distance=response)
            add('test1', 'response', offset, cfg, ('dwvp',) if response is None else ('dwvp', 'vp'), 'response_distance', response)
        add('test1', 'reference', offset, replace(config, fixed_lookahead=look), ('dwpp',))
    for look in settings['orientation_lookaheads']:
        for response in settings['orientation_responses']:
            add('test2', 'a', rapid, replace(config, fixed_lookahead=look, response_distance=response),
                parameter='response_distance', value=response)
    arc = dict(name='arc', kind='arc', radius=settings['arc_radius'], length=settings['arc_length'])
    for look in settings['response_lookaheads']:
        for response in settings['arc_responses']:
            add('test2', 'b', arc, replace(config, fixed_lookahead=look, response_distance=response),
                parameter='response_distance', value=response)
    # Reuse the unchanged v2 grid, excluding its auxiliary acceleration baseline.
    original, _ = plan_v2(config, settings, seed, ('test2', 'test3'))
    for c in original:
        c['test'] = {'test2': 'test3', 'test3': 'test4'}[c['test']]
        if c['test'] in selected and c['method'] != 'vp_scaled_accel':
            c['suite'] = 'access_v3'
            conditions.append(c)
    seeds = range(seed, seed + settings['noise_seeds'])
    for xy, yaw in settings['noise_levels']:
        cfg = replace(config, noise_xy=xy, noise_yaw=float(np.deg2rad(yaw)))
        for response in settings['noise_responses']:
            add('test5', 'b', offset, replace(cfg, fixed_lookahead=.11, response_distance=response),
                parameter='response_distance', value=response, seeds=seeds)
        for look in settings['fixed_lookaheads']:
            add('test5', 'b', offset, replace(cfg, fixed_lookahead=look),
                parameter='fixed_lookahead', value=look, seeds=seeds)
        add('test5', 'b', offset, cfg, ('dwpp', 'vp', 'vp_scaled'), 'adaptive', seeds=seeds)
        for scene in (dict(name='gradual', kind='ramp', transition_length=1.), rapid):
            add('test5', 'b', scene, cfg, methods, 'adaptive', seeds=seeds)
        add('test5', 'b', rapid, replace(cfg, fixed_lookahead=.11, response_distance=.33),
            parameter='response_distance', value=.33, seeds=seeds)
    for look in settings['fixed_lookaheads']:
        add('test5', 'c', rapid, replace(config, fixed_lookahead=look), methods, 'fixed_lookahead', look)
    return conditions


def run(study, output, config, settings, seed=0, force=False, config_path=None, workers=1, *unused):
    from .access_v3_outputs import outputs
    start = perf_counter()
    selected = selected_studies(study)
    output.mkdir(parents=True, exist_ok=True)
    if output.is_symlink():
        raise ValueError('output must be a real directory')
    conditions = plan_conditions(config, settings, seed, selected)
    numerical_source = numerical_hash()
    manifest = dict(schema_version=1, planner_version='access_v3.1', suite='access_v3', study=study,
                    source_sha256=code_hash(), numerical_source_sha256=numerical_source,
                    git_revision=git_revision(), python=sys.version, numpy=np.__version__, platform=platform.platform(),
                    cpu_model=cpu_model(), workers=workers, seed=seed, config=asdict(config), settings=settings,
                    dependency_lock_sha256=hashlib.sha256((ROOT/'uv.lock').read_bytes()).hexdigest(),
                    config_file_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                    planned_conditions=list(conditions), trials=[], complete=False)
    target = output/('manifest.json' if study == 'all' else f'manifest_{study}.json')
    if target.exists():
        previous = json.loads(target.read_text())
        manifest['previous_execution_attempts'] = previous.get('previous_execution_attempts', []) + [
            {k: previous.get(k) for k in ('source_sha256', 'complete', 'simulation_wall_time_s', 'total_wall_time_s')}]
    atomic_json(target, manifest)
    rows = []
    last_log = start
    def accept(c, result):
        nonlocal last_log
        row, spec, cached = result
        row.update(condition_id=sha(c), response_distance_m=c['config']['response_distance'])
        manifest['cache_hits'] = manifest.get('cache_hits', 0) + int(cached)
        rows.append(row)
        manifest['trials'].append(dict(condition_id=row['condition_id'], condition=c, spec=spec, summary=row))
        if perf_counter()-last_log >= 20 or len(rows) % 100 == 0:
            print(f"{len(rows)}/{len(manifest['planned_conditions'])} conditions; {sum(r['success'] for r in rows)} success; elapsed {perf_counter()-start:.1f}s", flush=True)
            atomic_json(target, manifest)
            last_log = perf_counter()
        return row
    if 'test5' in selected:
        rapid = dict(name='rapid', kind='ramp', transition_length=.3)
        def matched(method, scale, parameter):
            cfg = replace(config, vx_min=config.vx_min*scale, vx_max=config.vx_max*scale,
                          vy_min=config.vy_min*scale, vy_max=config.vy_max*scale,
                          vp_translation_speed=config.translation_speed*scale,
                          desired_linear_vel=config.desired_linear_vel*scale)
            c = make_condition('test5', 'a', rapid, method, cfg, seed, settings, parameter, scale)
            c['suite'] = 'access_v3'
            manifest['planned_conditions'].append(c)
            return accept(c, execute_trial(c, output, numerical_source, force))
        reference = matched('dwvp', 1., 'match_reference')
        manifest['time_matches'] = {}
        for method in ('vp', 'vp_scaled'):
            high = matched(method, 1., 'match_search')
            low = matched(method, settings['match_min_scale'], 'match_search')
            candidates = [high, low]
            lo, hi = settings['match_min_scale'], 1.
            target_time = reference.get('travel_time_s')
            bracket = target_time is not None and high.get('travel_time_s') is not None and high['travel_time_s'] <= target_time <= low.get('duration_s', 0)
            if bracket:
                for _ in range(settings['match_iterations']):
                    candidate = matched(method, (lo+hi)/2, 'match_search')
                    candidates.append(candidate)
                    if candidate.get('success') and abs(candidate['travel_time_s']-target_time) <= settings['match_tolerance_s']+1e-10:
                        break
                    if candidate.get('duration_s', float('inf')) > target_time:
                        lo = candidate['value']
                    else:
                        hi = candidate['value']
            successful = [r for r in candidates if r['success']]
            best = min(successful, key=lambda r: abs(r['travel_time_s']-target_time)) if successful and target_time is not None else None
            manifest['time_matches'][method] = dict(bracketed=bracket, target_time_s=target_time,
                best_trial_id=best['trial_id'] if best else None, best_scale=best['value'] if best else None,
                difference_s=best['travel_time_s']-target_time if best else None,
                matched=bool(best and abs(best['travel_time_s']-target_time)<=settings['match_tolerance_s']+1e-10))
    if workers == 1:
        for c in conditions:
            accept(c, execute_trial(c, output, numerical_source, force))
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            for c, result in zip(conditions, pool.map(execute_trial, conditions, repeat(output),
                                                      repeat(numerical_source), repeat(force), chunksize=1)):
                accept(c, result)
    for test in selected:
        directory = output/test
        directory.mkdir(exist_ok=True)
        write_csv(directory/'summary.csv', [r for r in rows if r['test'] == test])
    manifest.update(condition_entries=len(rows), distinct_trajectories=len({r['trial_id'] for r in rows}),
                    success_count=sum(r['success'] for r in rows), timeout_count=sum(r['status']=='timeout' for r in rows),
                    failure_count=sum(r['status'] not in ('success', 'timeout') for r in rows),
                    simulation_wall_time_s=perf_counter()-start)
    atomic_json(target, manifest)
    outputs(output, rows, manifest)
    manifest.update(complete=True, total_wall_time_s=perf_counter()-start)
    atomic_json(target, manifest)
    print(f"Finished {len(rows)} conditions / {manifest['distinct_trajectories']} trajectories in {manifest['total_wall_time_s']:.1f}s: "
          f"{manifest['success_count']} success, {manifest['timeout_count']} timeout, {manifest['failure_count']} failure", flush=True)
    return manifest
