"""Regenerate portable desired-velocity parity cases from the Python controller."""
import json
import math
from pathlib import Path
import numpy as np
from ec_dwvp.config import Config
from ec_dwvp.controller import desired_vector, preview_target
from ec_dwvp.paths import straight_path, arc_path


def generate():
    cases = []
    for geometry, base in [('straight', straight_path(.8,.1)), ('arc', arc_path(1., .8, .1))]:
        for rotation in (0., .7):
            path = base.copy()
            c, s = math.cos(rotation), math.sin(rotation)
            path[:, :2] = path[:, :2] @ np.array([[c,s],[-s,c]])
            path[:, 2] += rotation
            for response in (None, .165, .33):
                for region in ('chord','tangent','goal','endpoint'):
                    for sign in (-1,1):
                        for yaw in (.2, -2.4):
                            if region in ('chord', 'tangent'):
                                delta = path[4,:2]-path[3,:2]
                                tangent = delta / np.linalg.norm(delta)
                                normal = np.array([-tangent[1],tangent[0]])
                                xy = path[3,:2]+.023*tangent+sign*(.01 if region=='chord' else .5)*normal
                            else:
                                delta = path[-1,:2]-path[-2,:2]
                                tangent = delta/np.linalg.norm(delta)
                                normal = np.array([-tangent[1],tangent[0]])
                                xy = path[-1,:2]+(-.04 if region=='goal' else .03)*tangent+sign*.01*normal
                            pose = np.r_[xy,yaw]
                            cfg = Config(fixed_lookahead=.11,response_distance=response,vp_translation_speed=.22,
                                         orientation_time_weight=1.2 if yaw < 0 else 1.)
                            target, look, nearest, goal = preview_target(pose,np.zeros(3),path,cfg)
                            desired, rule = desired_vector(pose,target,cfg,path,goal_region=goal,return_rule=True)
                            cases.append(dict(id=f'{geometry}_{rotation}_{response}_{region}_{sign}_{yaw}',
                                pose=pose.tolist(),path=path.tolist(),lookahead_distance=look,response_distance=response,
                                translation_speed=cfg.translation_speed,orientation_time_weight=cfg.orientation_time_weight,
                                min_orientation_time=cfg.min_orientation_time,target=target.tolist(),goal_region=bool(goal),
                                nearest_pose=nearest,response_rule=rule,expected_desired_vector=desired.tolist()))
    return dict(schema_version=1, generator='tools/generate_desired_vector_fixtures.py',
                units='metres, seconds, radians; desired vector in body frame', cases=cases)


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    (root/'fixtures/desired_vector_cases.json').write_text(json.dumps(generate(),indent=2,allow_nan=False)+'\n')
