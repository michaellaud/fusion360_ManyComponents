"""Pure-Python core of the Transfer Kinematic mesh benchmark.

No `adsk` imports here, so it can be unit-tested outside Fusion.

Conventions used internally:
- Matrices are flat lists of 16 floats, row-major, translation at [3], [7], [11]
  (same layout as adsk.core.Matrix3D.asArray()).
- Lengths are in centimeters (Fusion internal unit).
- Track matrices are world transforms (root component context) after
  `finalize_tracks`.
"""

import bisect
import json
import math

IDENTITY = [1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0]

# Y-up (three.js / glTF) -> Z-up (Fusion default): rotation of +90 deg about X.
YUP_TO_ZUP = [1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, -1.0, 0.0,
              0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1.0]
ZUP_TO_YUP = [1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              0.0, -1.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1.0]


# ---------------------------------------------------------------------------
# Matrix helpers
# ---------------------------------------------------------------------------

def mat_mul(a, b):
    """Return a * b (row-major 4x4)."""
    r = [0.0] * 16
    for i in range(4):
        ai0, ai1, ai2, ai3 = a[4 * i], a[4 * i + 1], a[4 * i + 2], a[4 * i + 3]
        for j in range(4):
            r[4 * i + j] = ai0 * b[j] + ai1 * b[4 + j] + ai2 * b[8 + j] + ai3 * b[12 + j]
    return r


def mat_inverse(m):
    """General 4x4 inverse (Gauss-Jordan). Raises ValueError if singular."""
    a = [list(m[4 * i:4 * i + 4]) + [1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for col in range(4):
        piv = max(range(col, 4), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            raise ValueError('singular matrix')
        a[col], a[piv] = a[piv], a[col]
        p = a[col][col]
        a[col] = [v / p for v in a[col]]
        for r in range(4):
            if r != col:
                f = a[r][col]
                if f:
                    a[r] = [rv - f * cv for rv, cv in zip(a[r], a[col])]
    return [a[i][4 + j] for i in range(4) for j in range(4)]


def transpose(m):
    return [m[4 * j + i] for i in range(4) for j in range(4)]


def from_trs(pos, quat=None, scale=None):
    """Row-major matrix from translation, quaternion (x, y, z, w) and scale."""
    x, y, z, w = quat if quat is not None else (0.0, 0.0, 0.0, 1.0)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    sx, sy, sz = scale if scale is not None else (1.0, 1.0, 1.0)
    return [
        (1 - 2 * (y * y + z * z)) * sx, (2 * (x * y - z * w)) * sy, (2 * (x * z + y * w)) * sz, float(pos[0]),
        (2 * (x * y + z * w)) * sx, (1 - 2 * (x * x + z * z)) * sy, (2 * (y * z - x * w)) * sz, float(pos[1]),
        (2 * (x * z - y * w)) * sx, (2 * (y * z + x * w)) * sy, (1 - 2 * (x * x + y * y)) * sz, float(pos[2]),
        0.0, 0.0, 0.0, 1.0,
    ]


def from_euler_deg(pos, euler, order='XYZ'):
    """Row-major matrix from translation and Euler angles in degrees.

    `order` follows three.js semantics (intrinsic), e.g. 'XYZ' => R = Rx * Ry * Rz.
    """
    rots = {}
    for axis, ang in zip('XYZ', euler):
        c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        if axis == 'X':
            rots['X'] = [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1]
        elif axis == 'Y':
            rots['Y'] = [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1]
        else:
            rots['Z'] = [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    r = IDENTITY
    for axis in order.upper():
        r = mat_mul(r, rots[axis])
    r = list(r)
    r[3], r[7], r[11] = float(pos[0]), float(pos[1]), float(pos[2])
    return r


def scale_translation(m, k):
    r = list(m)
    r[3] *= k
    r[7] *= k
    r[11] *= k
    return r


def transform_points(m, coords):
    """Apply m to a flat [x0, y0, z0, x1, ...] list."""
    out = [0.0] * len(coords)
    m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11 = m[:12]
    for i in range(0, len(coords), 3):
        x, y, z = coords[i], coords[i + 1], coords[i + 2]
        out[i] = m0 * x + m1 * y + m2 * z + m3
        out[i + 1] = m4 * x + m5 * y + m6 * z + m7
        out[i + 2] = m8 * x + m9 * y + m10 * z + m11
    return out


def transform_normals(m, normals):
    """Rotate normals by the linear part of m (rigid assumed) and renormalize."""
    out = [0.0] * len(normals)
    m0, m1, m2, _, m4, m5, m6, _, m8, m9, m10 = m[:11]
    for i in range(0, len(normals), 3):
        x, y, z = normals[i], normals[i + 1], normals[i + 2]
        nx = m0 * x + m1 * y + m2 * z
        ny = m4 * x + m5 * y + m6 * z
        nz = m8 * x + m9 * y + m10 * z
        n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        out[i], out[i + 1], out[i + 2] = nx / n, ny / n, nz / n
    return out


def mat_close(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


# ---------------------------------------------------------------------------
# Motion JSON loading
# ---------------------------------------------------------------------------

class Motion(object):
    """Normalized trajectory.

    times: shared timeline (seconds) when `shared` is True, else None.
    tracks: {id: {'times': [...], 'mats': [[16], ...]}}
    """

    def __init__(self, tracks, meta=None):
        self.tracks = tracks
        self.meta = meta or {}
        ref = None
        shared = True
        for tr in tracks.values():
            if ref is None:
                ref = tr['times']
            elif tr['times'] != ref:
                shared = False
                break
        self.shared = shared and ref is not None
        self.times = ref if self.shared else None
        self.duration = max((tr['times'][-1] for tr in tracks.values() if tr['times']), default=0.0)
        self.start = min((tr['times'][0] for tr in tracks.values() if tr['times']), default=0.0)

    @property
    def frame_count(self):
        return max((len(tr['times']) for tr in self.tracks.values()), default=0)

    def indices_at(self, t):
        """Return {id: sample index} for simulation time t (last sample <= t)."""
        if self.shared:
            i = max(0, bisect.bisect_right(self.times, t) - 1)
            return {k: i for k in self.tracks}
        return {k: max(0, bisect.bisect_right(tr['times'], t) - 1) for k, tr in self.tracks.items()}

    def index_at(self, t):
        """Shared-timeline index (or None when timelines differ)."""
        if not self.shared:
            return None
        return max(0, bisect.bisect_right(self.times, t) - 1)


_FRAME_LIST_KEYS = ('frames', 'keyframes', 'samples', 'steps')
_FRAME_OBJ_KEYS = ('transforms', 'objects', 'parts', 'poses', 'occurrences', 'components', 'items', 'bodies')
_TRACK_KEYS = ('tracks', 'objects', 'parts', 'occurrences', 'components', 'animations', 'bodies', 'items')
_ID_KEYS = ('id', 'name', 'occurrence', 'fullPathName', 'path', 'key', 'uuid', 'entityToken')
_TIME_KEYS = ('time', 't', 'timestamp', 'sec', 'seconds')
_MATRIX_KEYS = ('matrix', 'transform', 'mat', 'm', 'matrixWorld', 'world')
_POS_KEYS = ('position', 'translation', 'pos', 't', 'p', 'xyz')
_QUAT_KEYS = ('quaternion', 'rotation', 'quat', 'q', 'rot')
_EULER_KEYS = ('euler', 'eulerDeg', 'rpy', 'angles')


def _first(d, keys):
    for k in keys:
        if k in d:
            return d[k]
    return None


def parse_transform(value, cfg):
    """Convert one transform value from the JSON into a row-major 16 list (JSON units)."""
    column_major = cfg.get('matrix_order', 'row') == 'column'
    if isinstance(value, dict):
        m = _first(value, _MATRIX_KEYS)
        if m is not None and not isinstance(m, dict):
            return parse_transform(m, cfg)
        pos = _first(value, _POS_KEYS)
        if pos is not None and not isinstance(pos, (int, float)):
            if isinstance(pos, dict):
                pos = [pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0)]
            quat = _first(value, _QUAT_KEYS)
            if isinstance(quat, dict):
                quat = [quat.get('x', 0.0), quat.get('y', 0.0), quat.get('z', 0.0), quat.get('w', 1.0)]
            euler = _first(value, _EULER_KEYS)
            if quat is not None and len(quat) == 4:
                if cfg.get('quaternion_order', 'xyzw') == 'wxyz':
                    quat = [quat[1], quat[2], quat[3], quat[0]]
                return from_trs(pos, quat, value.get('scale'))
            if quat is not None and len(quat) == 3:
                euler = quat
            if euler is not None:
                return from_euler_deg(pos, euler, cfg.get('euler_order', 'XYZ'))
            return from_trs(pos)
        raise ValueError('transform object without matrix/position: keys=%s' % sorted(value.keys()))
    if isinstance(value, (list, tuple)):
        if len(value) == 4 and all(isinstance(r, (list, tuple)) and len(r) == 4 for r in value):
            flat = [float(v) for r in value for v in r]
            return transpose(flat) if column_major else flat
        if len(value) == 16:
            flat = [float(v) for v in value]
            return transpose(flat) if column_major else flat
        if len(value) == 12:  # 3x4 without last row
            flat = [float(v) for v in value] + [0.0, 0.0, 0.0, 1.0]
            return flat
        if len(value) == 7:  # x y z qx qy qz qw
            return from_trs(value[:3], value[3:])
        if len(value) == 3:
            return from_trs(value)
    raise ValueError('unsupported transform value: %r' % (value,))


def _add_sample(tracks, oid, t, mat):
    tr = tracks.setdefault(str(oid), {'times': [], 'mats': []})
    tr['times'].append(float(t))
    tr['mats'].append(mat)


def _iter_objs(objs):
    """Yield (id, transform-value) from a dict or list container."""
    if isinstance(objs, dict):
        for k, v in objs.items():
            yield k, v
    elif isinstance(objs, list):
        for i, v in enumerate(objs):
            oid = _first(v, _ID_KEYS) if isinstance(v, dict) else None
            yield (oid if oid is not None else i), v


def parse_motion(data, cfg=None):
    """Detect and parse the motion JSON structure. Returns Motion (JSON units, JSON frame)."""
    cfg = cfg or {}
    meta = {}
    if isinstance(data, dict):
        for k in ('fps', 'frameRate', 'units', 'unit', 'upAxis', 'up', 'duration', 'version'):
            if k in data:
                meta[k] = data[k]
    fps = float(cfg.get('fps') or meta.get('fps') or meta.get('frameRate') or 30.0)
    tracks = {}
    fmt = None

    # Format 1: list of frames, each with a set of object transforms.
    frames = None
    if isinstance(data, list):
        frames = data
    elif isinstance(data, dict):
        frames = _first(data, _FRAME_LIST_KEYS)
    if isinstance(frames, list) and frames and isinstance(frames[0], dict) and \
            _first(frames[0], _FRAME_OBJ_KEYS) is not None:
        fmt = 'frames'
        for i, fr in enumerate(frames):
            t = _first(fr, _TIME_KEYS)
            if t is None:
                t = fr.get('frame', i) / fps
            for oid, v in _iter_objs(_first(fr, _FRAME_OBJ_KEYS)):
                _add_sample(tracks, oid, t, parse_transform(v, cfg))

    # Format 2: per-object tracks.
    if fmt is None and isinstance(data, dict):
        container = _first(data, _TRACK_KEYS)
        if container is not None:
            fmt = 'tracks'
            for oid, tr in _iter_objs(container):
                if not isinstance(tr, dict):
                    continue
                times = _first(tr, ('times', 'time', 't', 'timestamps', 'input'))
                mats = _first(tr, ('matrices', 'transforms', 'mats', 'poses', 'frames', 'keyframes', 'samples'))
                if mats is not None:
                    for i, v in enumerate(mats):
                        t = times[i] if times is not None else None
                        if t is None and isinstance(v, dict):
                            t = _first(v, _TIME_KEYS)
                        _add_sample(tracks, oid, t if t is not None else i / fps, parse_transform(v, cfg))
                    continue
                pos = _first(tr, ('positions', 'translations', 'translation', 'position'))
                rot = _first(tr, ('quaternions', 'rotations', 'rotation', 'quaternion'))
                n = len(pos if pos is not None else rot or [])
                for i in range(n):
                    v = {'position': pos[i] if pos is not None else [0, 0, 0]}
                    if rot is not None:
                        v['rotation'] = rot[i]
                    t = times[i] if times is not None else i / fps
                    _add_sample(tracks, oid, t, parse_transform(v, cfg))

    if not tracks:
        top = sorted(data.keys()) if isinstance(data, dict) else type(data).__name__
        raise ValueError('Estrutura de JSON nao reconhecida (chaves de topo: %s). '
                         'Configure "adapter_module" no config.json.' % (top,))
    for tr in tracks.values():
        order = sorted(range(len(tr['times'])), key=lambda i: tr['times'][i])
        tr['times'] = [tr['times'][i] for i in order]
        tr['mats'] = [tr['mats'][i] for i in order]
    meta['detected_format'] = fmt
    return Motion(tracks, meta)


def convert_units_and_axes(motion, cfg):
    """Scale translations to cm and convert up-axis in place. Returns motion."""
    k = float(cfg.get('unit_scale_to_cm', 1.0))
    axis = cfg.get('axis_conversion', 'none')
    if axis == 'yup_to_zup':
        c, ci = YUP_TO_ZUP, ZUP_TO_YUP
    elif axis == 'zup_to_yup':
        c, ci = ZUP_TO_YUP, YUP_TO_ZUP
    else:
        c = ci = None
    for tr in motion.tracks.values():
        mats = []
        for m in tr['mats']:
            if k != 1.0:
                m = scale_translation(m, k)
            if c is not None:
                m = mat_mul(mat_mul(c, m), ci)
            mats.append(m)
        tr['mats'] = mats
    return motion


def finalize_tracks(motion, initial_world, cfg):
    """Turn JSON transforms into absolute world matrices.

    transform_mode:
      'absolute'            JSON already has world matrices (default)
      'relative_to_initial' W = D * W0 (delta in world frame from initial pose)
      'incremental'         W_i = D_i * W_(i-1), starting from W0
    parent-relative data should be converted with an adapter.
    """
    mode = cfg.get('transform_mode', 'absolute')
    if mode == 'absolute':
        return motion
    for oid, tr in motion.tracks.items():
        w0 = initial_world.get(oid, IDENTITY)
        if mode == 'relative_to_initial':
            tr['mats'] = [mat_mul(d, w0) for d in tr['mats']]
        elif mode == 'incremental':
            cur, out = w0, []
            for d in tr['mats']:
                cur = mat_mul(d, cur)
                out.append(cur)
            tr['mats'] = out
        else:
            raise ValueError('transform_mode invalido: %s' % mode)
    return motion


def load_motion(path, cfg=None):
    cfg = cfg or {}
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    motion = parse_motion(data, cfg)
    return convert_units_and_axes(motion, cfg)


def describe_json(data, depth=0, max_depth=4, max_items=3):
    """Short schema description of a JSON document (for the inspect report)."""
    pad = '  ' * depth
    if depth > max_depth:
        return pad + '...\n'
    if isinstance(data, dict):
        out = ''
        for i, (k, v) in enumerate(data.items()):
            if i >= 12:
                out += pad + '... (%d chaves)\n' % len(data)
                break
            label = type(v).__name__ + ('[%d]' % len(v) if isinstance(v, (list, dict)) else '')
            if not isinstance(v, (list, dict)):
                label += ' = %r' % (v,) if len(repr(v)) < 60 else ''
            out += '%s%s: %s\n' % (pad, k, label)
            if isinstance(v, (dict, list)):
                out += describe_json(v, depth + 1, max_depth, max_items)
        return out
    if isinstance(data, list):
        if data and all(isinstance(v, (int, float)) for v in data[:32]):
            return pad + 'numeros: %s%s\n' % (data[:16], ' ...' if len(data) > 16 else '')
        out = ''
        for v in data[:1]:
            out += pad + '[0]: %s\n' % type(v).__name__
            out += describe_json(v, depth + 1, max_depth, max_items)
        return out
    return ''


# ---------------------------------------------------------------------------
# Occurrence matching
# ---------------------------------------------------------------------------

def match_ids(json_ids, occurrences, id_map=None):
    """Match JSON ids to occurrence descriptors.

    occurrences: list of dicts with keys 'fullPathName', 'name', 'componentName',
                 optionally 'entityToken'.
    Returns (matches {json_id: fullPathName}, unmatched [json_id], ambiguous {json_id: [paths]}).
    """
    id_map = id_map or {}
    by_path = {o['fullPathName']: o for o in occurrences}
    index = {}
    for o in occurrences:
        for key in ('fullPathName', 'name', 'entityToken'):
            v = o.get(key)
            if v:
                index.setdefault(('exact', v), []).append(o['fullPathName'])
        nm = o.get('name', '')
        if nm:
            index.setdefault(('lower', nm.lower()), []).append(o['fullPathName'])
            # "Part:1" and "Part_1" style exported names
            index.setdefault(('lower', nm.replace(':', '_').lower()), []).append(o['fullPathName'])
        path = o.get('fullPathName', '')
        if path:
            index.setdefault(('lower', path.replace('+', '/').lower()), []).append(o['fullPathName'])
            index.setdefault(('lower', path.replace(':', '_').replace('+', '/').lower()), []).append(o['fullPathName'])
        cn = o.get('componentName')
        if cn:
            index.setdefault(('comp', cn.lower()), []).append(o['fullPathName'])

    matches, unmatched, ambiguous = {}, [], {}
    for jid in json_ids:
        if jid in id_map:
            if id_map[jid] in by_path:
                matches[jid] = id_map[jid]
            else:
                unmatched.append(jid)
            continue
        found = None
        for key in (('exact', jid), ('lower', jid.lower()), ('comp', jid.lower())):
            cands = sorted(set(index.get(key, [])))
            if len(cands) == 1:
                found = cands[0]
                break
            if len(cands) > 1:
                ambiguous[jid] = cands
                break
        if found:
            matches[jid] = found
        elif jid not in ambiguous:
            unmatched.append(jid)
    return matches, unmatched, ambiguous


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def percentile(values, p):
    if not values:
        return 0.0
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def summarize(values_ms):
    if not values_ms:
        return {'n': 0, 'mean': 0.0, 'p95': 0.0, 'max': 0.0}
    return {
        'n': len(values_ms),
        'mean': sum(values_ms) / len(values_ms),
        'p95': percentile(values_ms, 95),
        'max': max(values_ms),
    }


class DropCounter(object):
    """Counts target ticks missed between presented frames."""

    def __init__(self, target_fps):
        self.target_fps = float(target_fps)
        self.last_tick = None
        self.dropped = 0

    def presented(self, elapsed_s):
        tick = int(math.floor(elapsed_s * self.target_fps + 1e-9))
        if self.last_tick is not None and tick > self.last_tick + 1:
            self.dropped += tick - self.last_tick - 1
        self.last_tick = tick
        return self.dropped


def sim_time(elapsed_s, motion_start, motion_duration, speed=1.0, loop=True):
    """Map wall time to simulation time (frames are skipped, never slowed down)."""
    span = motion_duration - motion_start
    t = elapsed_s * speed
    if span <= 0:
        return motion_start
    if loop:
        return motion_start + math.fmod(t, span)
    return motion_start + min(t, span)


def markdown_table(rows, columns):
    """rows: list of dicts; columns: list of (key, header, fmt)."""
    out = '| ' + ' | '.join(h for _, h, _ in columns) + ' |\n'
    out += '|' + '|'.join('---' for _ in columns) + '|\n'
    for r in rows:
        cells = []
        for key, _, fmt in columns:
            v = r.get(key, '')
            try:
                cells.append(fmt.format(v) if v != '' and v is not None else '-')
            except (ValueError, TypeError):
                cells.append(str(v))
        out += '| ' + ' | '.join(cells) + ' |\n'
    return out
