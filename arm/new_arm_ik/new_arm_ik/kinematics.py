import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    x, y, z = axis
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class Chain:
    def __init__(self, joints):
        self.joints = joints

    def fk(self, thetas):
        T = np.eye(4)
        for j, th in zip(self.joints, thetas):
            T_origin = homog(rpy_to_matrix(*j['origin_rpy']),
                             np.array(j['origin_xyz']))
            R_joint = axis_angle_matrix(j['axis'], th)
            T = T @ T_origin @ homog(R_joint, np.zeros(3))
        return T


def parse_urdf_chain(urdf_path, root_link, tip_link):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = {}
    for j in root.findall('joint'):
        jtype = j.get('type')
        if jtype == 'fixed':
            actuated = False
        else:
            actuated = True
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        origin = j.find('origin')
        if origin is not None:
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        else:
            xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        axis_el = j.find('axis')
        axis = [float(v) for v in axis_el.get('xyz').split()] if axis_el is not None else [0, 0, 1]
        limit = j.find('limit')
        if limit is not None:
            lo = float(limit.get('lower', -np.pi))
            hi = float(limit.get('upper', np.pi))
        else:
            lo, hi = -np.pi, np.pi
        joints[j.get('name')] = {
            'name': j.get('name'), 'type': jtype, 'parent': parent, 'child': child,
            'origin_xyz': xyz, 'origin_rpy': rpy, 'axis': axis,
            'lower': lo, 'upper': hi, 'actuated': actuated,
        }

    child_to_joint = {j['child']: j for j in joints.values()}
    chain = []
    current = tip_link
    while current != root_link:
        if current not in child_to_joint:
            raise RuntimeError(
                f'Cannot reach {root_link} from {tip_link}; stuck at {current}.')
        j = child_to_joint[current]
        chain.append(j)
        current = j['parent']
    chain.reverse()
    return Chain(chain)


class IKSolver:
    def __init__(self, chain, position_weight=1.0, regularization=1e-4,
                 tolerance=5e-4, restarts=6, seed=0):
        self.chain = chain
        self.n = len(chain.joints)
        self.actuated_idx = [i for i, j in enumerate(chain.joints) if j['actuated']]
        self.lower = np.array([j['lower'] for j in chain.joints])
        self.upper = np.array([j['upper'] for j in chain.joints])
        self.position_weight = position_weight
        # Small regularization keeps solutions near the seed (smooth motion in
        # RViz) without measurably hurting position accuracy.
        self.regularization = regularization
        # If the first (continuity-preserving) solve leaves a residual above
        # this tolerance, fall back to random restarts to escape local minima.
        self.tolerance = tolerance
        self.restarts = restarts
        self._rng = np.random.default_rng(seed)

    def _solve_once(self, target, q_init, q_ref, max_iter):
        """One least-squares solve, regularizing toward q_ref."""
        def residual(q):
            T = self.chain.fk(q)
            pos_err = self.position_weight * (T[:3, 3] - target)
            reg = self.regularization * (q - q_ref)
            return np.concatenate([pos_err, reg])

        result = least_squares(
            residual, q_init,
            bounds=(self.lower, self.upper),
            max_nfev=max_iter,
        )
        pos_err = self.chain.fk(result.x)[:3, 3] - target
        return result.x, float(np.linalg.norm(pos_err))

    def solve(self, target_xyz, q_init=None, max_iter=200):
        if q_init is None:
            q_init = np.zeros(self.n)
        q_init = np.clip(q_init, self.lower + 1e-6, self.upper - 1e-6)
        target = np.array(target_xyz, dtype=float)

        # First attempt: seed from (and regularize toward) the current pose so
        # the arm moves smoothly from where it is. This is the common case.
        best_q, best_err = self._solve_once(target, q_init, q_init, max_iter)
        if best_err <= self.tolerance:
            return best_q, best_err

        # Hard target: the continuity solve landed in a local minimum. Retry
        # from random seeds (no continuity bias) and keep the best result.
        for _ in range(self.restarts):
            seed = self._rng.uniform(self.lower, self.upper)
            q, err = self._solve_once(target, seed, seed, max_iter)
            if err < best_err:
                best_q, best_err = q, err
                if best_err <= self.tolerance:
                    break
        return best_q, best_err
