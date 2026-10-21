"""Shared geometric evaluation: segment projection and wrapped yaw interpolation."""
import numpy as np
from .geometry import wrap


def segment_tangents(path):
    """Unit tangents; duplicate segments inherit the nearest nonzero segment.

    Equal index-distance ties use the earlier segment; a constant path uses +x.
    Projection itself still uses the earliest closest segment, including duplicates.
    """
    delta = np.diff(np.asarray(path)[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    valid = np.flatnonzero(lengths > 1e-10)
    tangent = np.tile([1., 0.], (len(delta), 1))
    if len(valid):
        tangent[valid] = delta[valid] / lengths[valid, None]
        for i in np.flatnonzero(lengths <= 1e-10):
            tangent[i] = tangent[valid[np.argmin(np.abs(valid - i))]]
    return tangent


def signed_lateral_errors(poses, path, reference, segments):
    tangent = segment_tangents(path)[segments]
    offset = np.asarray(poses)[:, :2] - reference[:, :2]
    return -tangent[:, 1] * offset[:, 0] + tangent[:, 0] * offset[:, 1]


def project_reference(poses, path):
    """Closest XY point on polyline; yaw interpolated on that same segment.

    Ties use the earliest segment. Zero-length segments project to their first
    pose. Yaw interpolation follows the shortest signed angle in [-pi,pi).
    """
    poses=np.asarray(poses,dtype=float)
    path=np.asarray(path,dtype=float)
    starts=path[:-1,:2]
    delta=np.diff(path[:,:2],axis=0)
    lengths=np.sum(delta*delta,axis=1)
    references=[]
    indices=[]
    fractions=[]
    for start in range(0,len(poses),256):
        p=poses[start:start+256,:2]
        offset=p[:,None,:]-starts[None,:,:]
        numerator=np.sum(offset*delta[None,:,:],axis=2)
        alpha=np.divide(numerator,lengths[None,:],out=np.zeros_like(numerator),where=lengths[None,:]>1e-20)
        alpha=np.clip(alpha,0.,1.)
        projection=starts[None,:,:]+alpha[:,:,None]*delta[None,:,:]
        distance2=np.sum((p[:,None,:]-projection)**2,axis=2)
        nearest=np.argmin(distance2,axis=1)
        rows=np.arange(len(p))
        t=alpha[rows,nearest]
        xy=projection[rows,nearest]
        yaw=wrap(path[nearest,2]+t*wrap(path[nearest+1,2]-path[nearest,2]))
        references.append(np.column_stack((xy,yaw)))
        indices.append(nearest)
        fractions.append(t)
    reference=np.concatenate(references)
    errors=np.linalg.norm(poses[:,:2]-reference[:,:2],axis=1)
    yaw_errors=np.abs(wrap(poses[:,2]-reference[:,2]))
    return reference, errors, yaw_errors, np.concatenate(indices), np.concatenate(fractions)
