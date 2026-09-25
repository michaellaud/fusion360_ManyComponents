#Author-IA
#Description-Diagnostico: Transfer Kinematic - CAD (A), CAD em lote (B) e Custom Graphics (C)

"""Independent benchmark script for the Transfer Kinematic playback.

Runs as a Fusion *Script* (Scripts and Add-Ins > Scripts), never touches the
SmartToolsPro production flow. Everything is driven by config.json next to
this file. Always run it on a COPY of the assembly.
"""

import adsk.core, adsk.fusion, traceback
import os, sys, json, time, platform, importlib, datetime, ctypes

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import tk_core
importlib.reload(tk_core)

_app = None
_ui = None


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def _now():
    return time.perf_counter()


class _PMC(ctypes.Structure):
    _fields_ = [('cb', ctypes.c_ulong), ('PageFaultCount', ctypes.c_ulong),
                ('PeakWorkingSetSize', ctypes.c_size_t), ('WorkingSetSize', ctypes.c_size_t),
                ('QuotaPeakPagedPoolUsage', ctypes.c_size_t), ('QuotaPagedPoolUsage', ctypes.c_size_t),
                ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t), ('QuotaNonPagedPoolUsage', ctypes.c_size_t),
                ('PagefileUsage', ctypes.c_size_t), ('PeakPagefileUsage', ctypes.c_size_t)]


def memory_mb():
    """(working set MB, lifetime peak working set MB) of the Fusion process, or (None, None)."""
    try:
        pmc = _PMC()
        pmc.cb = ctypes.sizeof(_PMC)
        h = ctypes.windll.kernel32.GetCurrentProcess()
        if ctypes.windll.psapi.GetProcessMemoryInfo(h, ctypes.byref(pmc), pmc.cb):
            return pmc.WorkingSetSize / 1048576.0, pmc.PeakWorkingSetSize / 1048576.0
    except Exception:
        pass
    return None, None


def system_info():
    info = {
        'fusion_version': _app.version,
        'os': platform.platform(),
        'cpu': platform.processor(),
        'cpu_count': os.cpu_count(),
        'python': platform.python_version(),
    }
    try:
        class MS(ctypes.Structure):
            _fields_ = [('dwLength', ctypes.c_ulong), ('dwMemoryLoad', ctypes.c_ulong),
                        ('ullTotalPhys', ctypes.c_ulonglong), ('ullAvailPhys', ctypes.c_ulonglong),
                        ('ullTotalPageFile', ctypes.c_ulonglong), ('ullAvailPageFile', ctypes.c_ulonglong),
                        ('ullTotalVirtual', ctypes.c_ulonglong), ('ullAvailVirtual', ctypes.c_ulonglong),
                        ('ullAvailExtendedVirtual', ctypes.c_ulonglong)]
        ms = MS()
        ms.dwLength = ctypes.sizeof(MS)
        ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(ms))
        info['ram_gb'] = round(ms.ullTotalPhys / 1073741824.0, 1)
    except Exception:
        pass
    return info


def matrix3d(arr):
    m = adsk.core.Matrix3D.create()
    m.setWithArray(arr)
    return m


def load_config():
    path = os.path.join(_HERE, 'config.json')
    with open(path, 'r', encoding='utf-8') as f:
        cfg = json.load(f)
    cfg = {k: v for k, v in cfg.items() if not k.startswith('_')}
    return cfg


def resolve_motion_path(cfg):
    """Return an existing motion JSON path; ask with a file dialog and remember it in config.json."""
    path = cfg.get('motion_json') or ''
    if path and not os.path.isabs(path):
        path = os.path.join(_HERE, path)
    if path and os.path.isfile(path):
        return path
    dlg = _ui.createFileDialog()
    dlg.title = 'Selecione o JSON exportado pelo Transfer Kinematic'
    dlg.filter = 'JSON (*.json);;Todos (*.*)'
    dlg.isMultiSelectEnabled = False
    if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
        return None
    path = dlg.filename
    cfg_path = os.path.join(_HERE, 'config.json')
    try:
        with open(cfg_path, 'r', encoding='utf-8') as f:
            raw = json.load(f)
        raw['motion_json'] = path.replace('\\', '/')
        with open(cfg_path, 'w', encoding='utf-8') as f:
            json.dump(raw, f, indent=2, ensure_ascii=False)
    except Exception:
        pass
    return path


def load_motion(cfg):
    path = cfg['motion_json']
    if not os.path.isabs(path):
        path = os.path.join(_HERE, path)
    if cfg.get('adapter_module'):
        mod = importlib.import_module(cfg['adapter_module'])
        importlib.reload(mod)
        motion = mod.load(path, cfg)  # must return tk_core.Motion (JSON units)
        return tk_core.convert_units_and_axes(motion, cfg), path
    return tk_core.load_motion(path, cfg), path


class Log(object):
    def __init__(self, out_dir):
        self.lines = []
        self.path = os.path.join(out_dir, 'log.txt')

    def __call__(self, msg):
        line = '[%s] %s' % (datetime.datetime.now().strftime('%H:%M:%S'), msg)
        self.lines.append(line)
        with open(self.path, 'a', encoding='utf-8') as f:
            f.write(line + '\n')


# ---------------------------------------------------------------------------
# Binding JSON ids <-> occurrences
# ---------------------------------------------------------------------------

def occurrence_descriptors(root):
    descs, by_path = [], {}
    for occ in root.allOccurrences:
        d = {'fullPathName': occ.fullPathName, 'name': occ.name,
             'componentName': occ.component.name}
        try:
            d['entityToken'] = occ.entityToken
        except Exception:
            pass
        descs.append(d)
        by_path[occ.fullPathName] = occ
    return descs, by_path


def bind(motion, root, cfg, log):
    descs, by_path = occurrence_descriptors(root)
    ids = list(motion.tracks.keys())
    if cfg.get('only_ids'):
        ids = [i for i in ids if i in set(cfg['only_ids'])]
    matches, unmatched, ambiguous = tk_core.match_ids(ids, descs, cfg.get('id_map'))
    max_n = int(cfg.get('max_moving') or 0)
    selected = sorted(matches.items())
    if max_n > 0:
        selected = selected[:max_n]
    bound = [(jid, by_path[path]) for jid, path in selected]
    log('Ocorrencias no modelo: %d | ids no JSON: %d | casados: %d | sem par: %d | ambiguos: %d | usados: %d'
        % (len(descs), len(motion.tracks), len(matches), len(unmatched), len(ambiguous), len(bound)))
    if unmatched:
        log('Sem par (primeiros 20): %s' % unmatched[:20])
    for jid, c in list(ambiguous.items())[:20]:
        log('Ambiguo "%s": %s  -> use id_map no config.json' % (jid, c[:5]))
    return bound, descs, unmatched, ambiguous


# ---------------------------------------------------------------------------
# Mesh preparation for mode C
# ---------------------------------------------------------------------------

_QUALITY = {
    'low': 'LowQualityTriangleMesh',
    'normal': 'NormalQualityTriangleMesh',
    'high': 'HighQualityTriangleMesh',
    'veryhigh': 'VeryHighQualityTriangleMesh',
}


def appearance_rgb(body):
    try:
        ap = body.appearance
        for i in range(ap.appearanceProperties.count):
            p = adsk.core.ColorProperty.cast(ap.appearanceProperties.item(i))
            if p and p.value:
                c = p.value
                return (c.red, c.green, c.blue, c.opacity)
    except Exception:
        pass
    return (170, 170, 170, 255)


def triangulate_component(comp, local, skip_rel, rel, quality, buckets, stats):
    """Accumulate triangles of comp (and visible children) into buckets keyed by color,
    expressed in the moving occurrence's own component frame."""
    q = getattr(adsk.fusion.TriangleMeshQualityOptions, _QUALITY.get(quality, _QUALITY['normal']))
    for body in comp.bRepBodies:
        if not body.isLightBulbOn:
            continue
        calc = body.meshManager.createMeshCalculator()
        calc.setQuality(q)
        mesh = calc.calculate()
        coords = list(mesh.nodeCoordinatesAsDouble)
        normals = list(mesh.normalVectorsAsDouble)
        idx = list(mesh.nodeIndices)
        if local is not None:
            coords = tk_core.transform_points(local, coords)
            normals = tk_core.transform_normals(local, normals)
        b = buckets.setdefault(appearance_rgb(body), {'coords': [], 'normals': [], 'idx': []})
        base = len(b['coords']) // 3
        b['coords'].extend(coords)
        b['normals'].extend(normals)
        b['idx'].extend(i + base for i in idx)
        stats['bodies'] += 1
        stats['triangles'] += len(idx) // 3
    for child in comp.occurrences:
        if not child.isLightBulbOn:
            continue
        child_rel = child.name if not rel else rel + '+' + child.name
        if child_rel in skip_rel:
            continue  # animated separately
        m = list(child.transform2.asArray())
        m = m if local is None else tk_core.mat_mul(local, m)
        triangulate_component(child.component, m, skip_rel, child_rel, quality, buckets, stats)


class MeshScene(object):
    """Custom graphics for the moving occurrences. Created once, only transforms change."""

    def __init__(self, root, bound, cfg, log):
        self.root = root
        self.cfg = cfg
        self.log = log
        self.group = None
        self.instances = []  # (json_id, [meshes], Matrix3D)
        self.stats = {'bodies': 0, 'triangles': 0, 'meshes': 0, 'unique_geometries': 0,
                      'triangulation_s': 0.0, 'creation_s': 0.0, 'instances': 0,
                      'drawn_triangles': 0}
        self._build(bound)

    def _build(self, bound):
        quality = self.cfg.get('mesh_quality', 'normal')
        moving_paths = set(o.fullPathName for _, o in bound)
        cache = {}
        t0 = _now()
        per_instance = []
        for jid, occ in bound:
            prefix = occ.fullPathName + '+'
            skip = frozenset(p[len(prefix):] for p in moving_paths if p.startswith(prefix))
            key = (occ.component.entityToken if hasattr(occ.component, 'entityToken') else occ.component.name, skip)
            if key not in cache:
                buckets = {}
                s = {'bodies': 0, 'triangles': 0}
                triangulate_component(occ.component, None, skip, '', quality, buckets, s)
                cache[key] = (buckets, s)
                self.stats['bodies'] += s['bodies']
                self.stats['triangles'] += s['triangles']
            per_instance.append((jid, occ, key))
        self.stats['triangulation_s'] = _now() - t0
        self.stats['unique_geometries'] = len(cache)

        t1 = _now()
        self.group = self.root.customGraphicsGroups.add()
        coord_objs = {}
        for jid, occ, key in per_instance:
            buckets, s = cache[key]
            meshes = []
            for rgba, b in buckets.items():
                if not b['idx']:
                    continue
                ck = (key, rgba)
                if ck not in coord_objs:
                    coord_objs[ck] = adsk.fusion.CustomGraphicsCoordinates.create(b['coords'])
                try:
                    mesh = self.group.addMesh(coord_objs[ck], b['idx'], b['normals'], b['idx'])
                except Exception:
                    # Coordinates object may not be shareable between meshes on some builds.
                    coords = adsk.fusion.CustomGraphicsCoordinates.create(b['coords'])
                    mesh = self.group.addMesh(coords, b['idx'], b['normals'], b['idx'])
                r, g, bl, a = rgba
                col = adsk.core.Color.create(r, g, bl, 255)
                mesh.color = adsk.fusion.CustomGraphicsBasicMaterialColorEffect.create(
                    col, col, adsk.core.Color.create(40, 40, 40, 255),
                    adsk.core.Color.create(0, 0, 0, 255), 20.0, a / 255.0)
                mesh.isSelectable = False
                meshes.append(mesh)
                self.stats['drawn_triangles'] += len(b['idx']) // 3
            mat = matrix3d(list(occ.transform2.asArray()))
            for mesh in meshes:
                mesh.transform = mat
            self.instances.append((jid, meshes, mat))
            self.stats['meshes'] += len(meshes)
        self.stats['instances'] = len(self.instances)
        self.stats['creation_s'] = _now() - t1

    def apply(self, mats_by_id):
        for jid, meshes, mat in self.instances:
            arr = mats_by_id.get(jid)
            if arr is None:
                continue
            mat.setWithArray(arr)
            for mesh in meshes:
                mesh.transform = mat

    def delete(self):
        if self.group and self.group.isValid:
            self.group.deleteMe()
        self.group = None


# ---------------------------------------------------------------------------
# Frame appliers
# ---------------------------------------------------------------------------

class CadIndividual(object):
    """Mode A. Mirrors the usual add-in pattern: one transform2 assignment per occurrence.
    If Transfer Kinematic uses a different call sequence, put it in apply()."""

    def __init__(self, bound):
        self.items = [(jid, occ, adsk.core.Matrix3D.create()) for jid, occ in bound]

    def apply(self, mats_by_id):
        for jid, occ, m in self.items:
            arr = mats_by_id.get(jid)
            if arr is not None:
                m.setWithArray(arr)
                occ.transform2 = m


class CadBatch(object):
    """Mode B. One Component.transformOccurrences call per frame."""

    def __init__(self, root, bound, ignore_joints):
        self.root = root
        self.ignore_joints = ignore_joints
        self.ids = [jid for jid, _ in bound]
        self.occs = [occ for _, occ in bound]
        self.mats = [adsk.core.Matrix3D.create() for _ in bound]

    def apply(self, mats_by_id):
        for i, jid in enumerate(self.ids):
            arr = mats_by_id.get(jid)
            if arr is not None:
                self.mats[i].setWithArray(arr)
        ok = self.root.transformOccurrences(self.occs, self.mats, self.ignore_joints)
        if not ok:
            raise RuntimeError('transformOccurrences retornou False')


# ---------------------------------------------------------------------------
# Playback + measurement
# ---------------------------------------------------------------------------

def frame_mats(motion, t):
    idx = motion.indices_at(t)
    return {k: motion.tracks[k]['mats'][i] for k, i in idx.items()}


def play(mode, applier, motion, cfg, vp, progress, log, frames_csv):
    target_fps = float(cfg.get('target_fps', 30))
    warmup = float(cfg.get('warmup_s', 3))
    duration = float(cfg.get('duration_s', 30))
    speed = float(cfg.get('speed', 1.0))
    update_every = max(1, int(cfg.get('update_every_n_frames', 1)))
    do_refresh = bool(cfg.get('explicit_refresh', True))
    pace = bool(cfg.get('pace_to_target', True))
    period = 1.0 / target_fps

    samples = {'query': [], 'apply': [], 'refresh': [], 'events': [], 'work': [], 'interval': []}
    drops = tk_core.DropCounter(target_fps)
    mem_max = None
    presented = 0
    last_idx = None
    repeated = 0

    t_start = _now()
    t_measure = t_start + warmup
    t_end = t_measure + duration
    prev_frame_start = None
    frame_no = 0
    rows = []
    while True:
        f0 = _now()
        if f0 >= t_end:
            break
        measuring = f0 >= t_measure
        elapsed = f0 - t_start
        t_sim = tk_core.sim_time(elapsed, motion.start, motion.duration, speed, cfg.get('loop', True))

        mats = frame_mats(motion, t_sim)
        f1 = _now()
        if frame_no % update_every == 0:
            applier.apply(mats)
        f2 = _now()
        if do_refresh:
            vp.refresh()
        f3 = _now()
        adsk.doEvents()
        f4 = _now()

        idx = motion.index_at(t_sim)
        if idx is not None and idx == last_idx:
            repeated += 1
        last_idx = idx

        if measuring:
            samples['query'].append((f1 - f0) * 1000)
            samples['apply'].append((f2 - f1) * 1000)
            samples['refresh'].append((f3 - f2) * 1000)
            samples['events'].append((f4 - f3) * 1000)
            samples['work'].append((f4 - f0) * 1000)
            if prev_frame_start is not None and prev_frame_start >= t_measure:
                samples['interval'].append((f0 - prev_frame_start) * 1000)
            drops.presented(f0 - t_measure)
            presented += 1
            rows.append('%s,%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f\n' % (
                mode, frame_no, t_sim, (f1 - f0) * 1000, (f2 - f1) * 1000,
                (f3 - f2) * 1000, (f4 - f3) * 1000, (f4 - f0) * 1000))
            if presented % 30 == 0:
                ws, _ = memory_mb()
                if ws is not None:
                    mem_max = ws if mem_max is None else max(mem_max, ws)
        prev_frame_start = f0
        frame_no += 1

        if progress:
            progress.progressValue = int(min(100, 100 * (f0 - t_start) / (warmup + duration)))
            if progress.wasCancelled:
                raise KeyboardInterrupt('cancelado pelo usuario')

        if pace:
            next_t = f0 + period
            while _now() < next_t:
                adsk.doEvents()
                time.sleep(0.0005)

    with open(frames_csv, 'a', encoding='utf-8') as f:
        f.writelines(rows)

    measured_s = max(1e-9, min(_now(), t_end) - t_measure)
    res = {'mode': mode, 'frames': presented, 'fps': presented / measured_s,
           'dropped': drops.dropped, 'repeated_sim_frames': repeated,
           'mem_ws_max_mb': mem_max}
    for k, v in samples.items():
        s = tk_core.summarize(v)
        res[k + '_mean'] = s['mean']
        res[k + '_p95'] = s['p95']
        res[k + '_max'] = s['max']
    log('%s: %.1f FPS | intervalo medio %.1f ms p95 %.1f max %.1f | apply medio %.2f ms | refresh %.2f ms | doEvents %.2f ms | perdidos %d'
        % (mode, res['fps'], res['interval_mean'], res['interval_p95'], res['interval_max'],
           res['apply_mean'], res['refresh_mean'], res['events_mean'], res['dropped']))
    return res


def snapshots(mode, applier, motion, cfg, vp, out_dir):
    fracs = cfg.get('snapshot_fractions') or []
    w, h = cfg.get('snapshot_size', [0, 0])
    for fr in fracs:
        t = motion.start + (motion.duration - motion.start) * float(fr)
        applier.apply(frame_mats(motion, t))
        vp.refresh()
        adsk.doEvents()
        vp.saveAsImageFile(os.path.join(out_dir, '%s_t%.3fs.png' % (mode, t)), int(w), int(h))


# ---------------------------------------------------------------------------
# State save / restore
# ---------------------------------------------------------------------------

class SavedState(object):
    def __init__(self, design, bound, fixed_occs, vp):
        self.design = design
        self.vp = vp
        self.camera = vp.camera
        self.transforms = [(occ, list(occ.transform2.asArray())) for _, occ in bound]
        self.bulbs = [(occ, occ.isLightBulbOn) for _, occ in bound]
        self.fixed_bulbs = [(occ, occ.isLightBulbOn) for occ in fixed_occs]

    def restore_camera(self):
        cam = self.camera
        cam.isSmoothTransition = False
        self.vp.camera = cam

    def restore(self, log=None):
        t0 = _now()
        errors = 0
        for occ, arr in self.transforms:
            try:
                occ.transform2 = matrix3d(arr)
            except Exception:
                errors += 1
        for occ, on in self.bulbs + self.fixed_bulbs:
            try:
                if occ.isLightBulbOn != on:
                    occ.isLightBulbOn = on
            except Exception:
                errors += 1
        try:
            snaps = self.design.snapshots
            if snaps.hasPendingSnapshot and hasattr(snaps, 'revertPendingSnapshot'):
                snaps.revertPendingSnapshot()
        except Exception:
            pass
        self.vp.refresh()
        adsk.doEvents()
        dt = _now() - t0
        if log:
            log('Restauracao: %.3f s (%d erros)' % (dt, errors))
        return dt


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run_benchmark(cfg, out_dir, log):
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if not design:
        raise RuntimeError('Abra um design do Fusion (copia da montagem de teste).')
    doc_name = _app.activeDocument.name
    need = cfg.get('require_doc_name_contains')
    if need and need.lower() not in doc_name.lower():
        raise RuntimeError('Documento "%s" nao contem "%s". Rode apenas numa COPIA.' % (doc_name, need))

    root = design.rootComponent
    vp = _app.activeViewport
    info = system_info()
    info.update({'document': doc_name, 'viewport': [vp.width, vp.height],
                 'design_type': 'parametric' if design.designType == adsk.fusion.DesignTypes.ParametricDesignType else 'direct'})
    log('Sistema: %s' % json.dumps(info, ensure_ascii=False))

    t0 = _now()
    try:
        motion, json_path = load_motion(cfg)
    except ValueError as e:
        # Unknown structure: still dump what is needed to write an adapter.
        with open(cfg['motion_json'], 'r', encoding='utf-8') as jf:
            raw = json.load(jf)
        with open(os.path.join(out_dir, 'json_schema.txt'), 'w', encoding='utf-8') as f:
            f.write(tk_core.describe_json(raw, max_depth=6))
        descs, _ = occurrence_descriptors(root)
        with open(os.path.join(out_dir, 'occurrences.txt'), 'w', encoding='utf-8') as f:
            for d in descs:
                f.write('%s\t%s\n' % (d['fullPathName'], d['componentName']))
        log(str(e))
        log('Gravados json_schema.txt e occurrences.txt para criar o adaptador.')
        raise RuntimeError('%s\n\nForam gravados json_schema.txt e occurrences.txt em:\n%s' % (e, out_dir))
    log('JSON: %s | formato %s | %d trilhas | %d amostras | %.3f..%.3f s | timeline compartilhada: %s | meta %s | carga %.3f s'
        % (json_path, motion.meta.get('detected_format'), len(motion.tracks), motion.frame_count,
           motion.start, motion.duration, motion.shared, motion.meta, _now() - t0))

    bound, descs, unmatched, ambiguous = bind(motion, root, cfg, log)
    initial = {jid: list(occ.transform2.asArray()) for jid, occ in bound}
    tk_core.finalize_tracks(motion, initial, cfg)

    # Sanity check: first JSON pose vs current CAD pose (reveals unit/axis/space errors).
    worst = 0.0
    for jid, occ in bound[:50]:
        m0 = motion.tracks[jid]['mats'][0]
        c0 = initial[jid]
        d = ((m0[3] - c0[3]) ** 2 + (m0[7] - c0[7]) ** 2 + (m0[11] - c0[11]) ** 2) ** 0.5
        worst = max(worst, d)
    log('Diferenca maxima entre pose inicial do JSON e pose CAD atual (50 primeiros): %.4f cm '
        '(valores grandes indicam unidade/eixo/referencial errado ou montagem fora da pose inicial)' % worst)

    report = {'system': info, 'config': cfg, 'json': {
        'path': json_path, 'format': motion.meta.get('detected_format'), 'tracks': len(motion.tracks),
        'samples': motion.frame_count, 'start_s': motion.start, 'end_s': motion.duration,
        'shared_timeline': motion.shared, 'meta': motion.meta},
        'binding': {'model_occurrences': len(descs), 'bound': len(bound),
                    'unmatched': unmatched[:200], 'ambiguous': dict(list(ambiguous.items())[:200])},
        'initial_pose_max_diff_cm': worst, 'results': [], 'mesh': None}

    if cfg.get('inspect_only', False) or not bound:
        with open(os.path.join(out_dir, 'json_schema.txt'), 'w', encoding='utf-8') as f:
            with open(json_path, 'r', encoding='utf-8') as jf:
                f.write(tk_core.describe_json(json.load(jf)))
        with open(os.path.join(out_dir, 'occurrences.txt'), 'w', encoding='utf-8') as f:
            for d in descs:
                f.write('%s\t%s\n' % (d['fullPathName'], d['componentName']))
        if not bound:
            log('Nenhuma ocorrencia casada. Veja occurrences.txt e json_schema.txt e preencha id_map.')
        return report

    moving_paths = set(o.fullPathName for _, o in bound)
    fixed = [o for o in root.occurrences if o.fullPathName not in moving_paths
             and not any(p.startswith(o.fullPathName + '+') for p in moving_paths)]
    state = SavedState(design, bound, fixed, vp)
    frames_csv = os.path.join(out_dir, 'frames.csv')
    with open(frames_csv, 'w', encoding='utf-8') as f:
        f.write('mode,frame,t_sim,query_ms,apply_ms,refresh_ms,doevents_ms,work_ms\n')

    progress = None
    if cfg.get('show_progress', True):
        progress = _ui.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.show('TK Mesh Benchmark', 'Medindo...', 0, 100)

    scene = None
    try:
        if cfg.get('hide_fixed', False):
            for o in fixed:
                o.isLightBulbOn = False

        for mode in cfg.get('modes', ['A', 'B', 'C']):
            state.restore_camera()
            mem0, _ = memory_mb()
            prep0 = _now()
            if mode == 'A':
                applier = CadIndividual(bound)
            elif mode == 'B':
                applier = CadBatch(root, bound, bool(cfg.get('ignore_joints', True)))
            elif mode == 'C':
                if scene is None:
                    scene = MeshScene(root, bound, cfg, log)
                    ws, _ = memory_mb()
                    scene.stats['mem_delta_mb'] = (ws - mem0) if (ws is not None and mem0 is not None) else None
                    report['mesh'] = scene.stats
                    log('Malhas: %s' % json.dumps(scene.stats))
                for _, occ in bound:
                    occ.isLightBulbOn = False
                applier = scene
            else:
                log('Modo desconhecido: %s' % mode)
                continue
            prep_s = _now() - prep0
            if progress:
                progress.message = 'Modo %s (%%p%%)' % mode
            try:
                res = play(mode, applier, motion, cfg, vp, progress, log, frames_csv)
            except KeyboardInterrupt:
                log('Cancelado durante o modo %s' % mode)
                break
            except Exception:
                log('Erro no modo %s:\n%s' % (mode, traceback.format_exc()))
                res = {'mode': mode, 'error': traceback.format_exc().splitlines()[-1]}
            res['prep_s'] = prep_s
            res['occurrences'] = len(bound)
            res['triangles'] = scene.stats['drawn_triangles'] if (mode == 'C' and scene) else None
            ws, peak = memory_mb()
            res['mem_ws_end_mb'] = ws
            res['mem_peak_lifetime_mb'] = peak
            if 'error' not in res:
                try:
                    snapshots(mode, applier, motion, cfg, vp, out_dir)
                except Exception:
                    log('Snapshot falhou: %s' % traceback.format_exc().splitlines()[-1])
            t_stop = _now()
            if mode == 'C':
                for occ, on in state.bulbs:
                    occ.isLightBulbOn = on
            res['reset_s'] = state.restore(log) + (_now() - t_stop)
            report['results'].append(res)
    finally:
        t_c = _now()
        if scene:
            scene.delete()
        state.restore(log)
        state.restore_camera()
        report['cleanup_s'] = _now() - t_c
        if progress:
            progress.hide()
    return report


def write_outputs(report, out_dir):
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=str)
    cols = [('mode', 'Modo', '{}'), ('occurrences', 'Ocorr.', '{}'), ('triangles', 'Triangulos', '{}'),
            ('fps', 'FPS', '{:.1f}'), ('interval_mean', 'Quadro medio ms', '{:.1f}'),
            ('interval_p95', 'p95 ms', '{:.1f}'), ('interval_max', 'max ms', '{:.1f}'),
            ('apply_mean', 'Aplicar ms', '{:.2f}'), ('refresh_mean', 'refresh() ms', '{:.2f}'),
            ('events_mean', 'doEvents ms', '{:.2f}'), ('dropped', 'Perdidos', '{}'),
            ('prep_s', 'Preparo s', '{:.2f}'), ('reset_s', 'Reset s', '{:.2f}'),
            ('mem_ws_max_mb', 'Mem max MB', '{:.0f}'), ('error', 'Erro', '{}')]
    md = '# TK Mesh Benchmark - %s\n\n' % os.path.basename(out_dir)
    md += '## Configuracao\n\n```\n%s\n```\n\n' % json.dumps(report.get('system'), indent=2, ensure_ascii=False)
    md += '## Resultados A/B/C\n\n' + tk_core.markdown_table(report.get('results', []), cols)
    if report.get('mesh'):
        md += '\n## Malhas (modo C)\n\n```\n%s\n```\n' % json.dumps(report['mesh'], indent=2)
    md += '\nAlvo: %s FPS | aquecimento %s s | duracao %s s | ignore_joints=%s | qualidade=%s\n' % (
        report['config'].get('target_fps'), report['config'].get('warmup_s'),
        report['config'].get('duration_s'), report['config'].get('ignore_joints'),
        report['config'].get('mesh_quality'))
    md += ('\nObs.: "refresh() ms" e "doEvents ms" nao sao o custo total de renderizacao; '
           'use "Quadro medio"/p95 (intervalo entre inicios de quadro) como medida principal.\n')
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    return md


def run(context):
    global _app, _ui
    _app = adsk.core.Application.get()
    _ui = _app.userInterface
    log = None
    try:
        cfg = load_config()
        motion_path = resolve_motion_path(cfg)
        if not motion_path:
            return
        cfg['motion_json'] = motion_path
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(cfg.get('results_dir') or os.path.join(_HERE, 'results'), stamp)
        os.makedirs(out_dir, exist_ok=True)
        log = Log(out_dir)
        if cfg.get('confirm', True):
            r = _ui.messageBox(
                'Benchmark Transfer Kinematic\n\nDocumento: %s\nModos: %s\nDuracao: %s s (+%s s aquecimento) por modo\n\n'
                'Use apenas numa COPIA da montagem. Posicoes e visibilidade serao restauradas ao final.\n\nContinuar?'
                % (_app.activeDocument.name, cfg.get('modes'), cfg.get('duration_s'), cfg.get('warmup_s')),
                'TK Mesh Benchmark', adsk.core.MessageBoxButtonTypes.YesNoButtonType)
            if r != adsk.core.DialogResults.DialogYes:
                return
        report = run_benchmark(cfg, out_dir, log)
        md = write_outputs(report, out_dir)
        _ui.messageBox('Concluido. Resultados em:\n%s\n\n%s' % (out_dir, md[-1500:]), 'TK Mesh Benchmark')
    except Exception:
        msg = traceback.format_exc()
        if log:
            log(msg)
        if _ui:
            _ui.messageBox('Falha:\n%s' % msg)
