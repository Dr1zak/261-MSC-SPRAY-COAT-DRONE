"""Pure Python validation, collision-free routing, marking and reporting.

The deterministic backend in this module is for tests and algorithm experiments;
it does not model flight physics or claim Isaac Sim validation.
"""
from __future__ import annotations

import csv
import hashlib
import heapq
import json
import math
import random
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Point:
    point_id: str
    x: float
    y: float
    z: float
    mark_type: str
    priority: int

    @property
    def xy(self):
        return (self.x, self.y)


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    required = ['seed', 'bounds', 'home', 'layout', 'robot_radius', 'clearance',
                'grid_resolution', 'speed', 'mark_tolerance', 'max_retries']
    if not isinstance(config, dict) or any(k not in config for k in required):
        raise ValueError('scenario missing required configuration fields')
    bounds = config['bounds']
    if len(bounds) != 4 or not all(math.isfinite(float(v)) for v in bounds) or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError('invalid bounds [xmin,xmax,ymin,ymax]')
    for key in ('robot_radius', 'clearance', 'grid_resolution', 'speed', 'mark_tolerance'):
        if not math.isfinite(float(config[key])) or float(config[key]) <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if int(config['max_retries']) < 0 or int(config['seed']) < 0:
        raise ValueError('seed and max_retries must be nonnegative')
    if len(config['home']) != 2 or not inside(tuple(config['home']), bounds):
        raise ValueError('home outside site')
    for obstacle in config.get('obstacles', []):
        if len(obstacle) != 4 or obstacle[0] >= obstacle[1] or obstacle[2] >= obstacle[3]:
            raise ValueError('invalid rectangular obstacle')
    return config


def inside(p, bounds):
    return bounds[0] <= p[0] <= bounds[1] and bounds[2] <= p[1] <= bounds[3]


def load_layout(path, bounds):
    valid, rejected, ids = [], [], set()
    required = {'point_id', 'x_m', 'y_m', 'z_m', 'mark_type', 'priority'}
    with open(path, newline='') as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f'layout requires columns {sorted(required)}')
        for line, row in enumerate(reader, 2):
            try:
                name = row['point_id'].strip()
                if not name or name in ids:
                    raise ValueError('empty or duplicate point_id')
                ids.add(name)
                xyz = [float(row[k]) for k in ('x_m', 'y_m', 'z_m')]
                if not all(math.isfinite(v) for v in xyz):
                    raise ValueError('non-finite coordinate')
                if not inside(xyz, bounds):
                    raise ValueError('outside site')
                if abs(xyz[2]) > 1e-6:
                    raise ValueError('unsupported ground elevation')
                if row['mark_type'] != 'dot':
                    raise ValueError('unsupported mark type')
                if not row['priority'].isdigit() or int(row['priority']) < 1:
                    raise ValueError('priority must be a positive integer')
                valid.append(Point(name, *xyz, row['mark_type'], int(row['priority'])))
            except (ValueError, TypeError, KeyError) as exc:
                rejected.append({'line': line, 'point_id': row.get('point_id', ''), 'reason': str(exc)})
    return valid, rejected


class GridPlanner:
    """Conservative 8-connected A*; rectangles are columns at all altitudes."""

    def __init__(self, bounds, obstacles, radius, clearance, resolution):
        self.bounds, self.resolution = bounds, resolution
        pad = radius + clearance
        self.boundary_pad = pad
        self.obstacles = [(a-pad, b+pad, c-pad, d+pad) for a,b,c,d in obstacles]
        self.nx = math.floor((bounds[1]-bounds[0])/resolution)+1
        self.ny = math.floor((bounds[3]-bounds[2])/resolution)+1
        self.cache = {}

    def free(self, p):
        b = self.bounds
        pad = self.boundary_pad
        return (b[0]+pad <= p[0] <= b[1]-pad and b[2]+pad <= p[1] <= b[3]-pad
                and not any(xmin <= p[0] <= xmax and ymin <= p[1] <= ymax
                            for xmin,xmax,ymin,ymax in self.obstacles))

    def segment_free(self, a, b):
        n = max(1, math.ceil(math.dist(a,b)/(self.resolution/3)))
        return all(self.free((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)) for i in range(n+1))

    def path(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        if not self.free(start) or not self.free(goal):
            return None
        if self.segment_free(start, goal):
            return [start, goal]
        key = (start, goal)
        if key in self.cache:
            return self.cache[key]
        def cell(p):
            return (round((p[0]-self.bounds[0])/self.resolution), round((p[1]-self.bounds[2])/self.resolution))
        def pos(c):
            return (self.bounds[0]+c[0]*self.resolution, self.bounds[2]+c[1]*self.resolution)
        source, dest = cell(start), cell(goal)
        queue = [(0.0, 0.0, source)]
        cost, parent = {source: 0.0}, {}
        directions = [(x,y) for x in (-1,0,1) for y in (-1,0,1) if (x,y)!=(0,0)]
        while queue:
            _, g, node = heapq.heappop(queue)
            if g > cost[node]+1e-9:
                continue
            if node == dest:
                cells = [node]
                while cells[-1] != source:
                    cells.append(parent[cells[-1]])
                raw = [start]+[pos(c) for c in reversed(cells)]+[goal]
                if not all(self.segment_free(a,b) for a,b in zip(raw,raw[1:])):
                    self.cache[key]=None
                    return None
                result = [raw[0]]
                for p in raw[1:]:
                    if len(result)>1 and self.segment_free(result[-2],p):
                        result[-1]=p
                    else:
                        result.append(p)
                if not all(self.segment_free(a,b) for a,b in zip(result,result[1:])):
                    raise AssertionError('planner emitted a colliding segment')
                self.cache[key]=result
                return result
            for dx,dy in directions:
                nxt=(node[0]+dx,node[1]+dy)
                if not (0 <= nxt[0] < self.nx and 0 <= nxt[1] < self.ny):
                    continue
                if not self.segment_free(pos(node),pos(nxt)):
                    continue
                ng=g+math.hypot(dx,dy)*self.resolution
                if ng+1e-9<cost.get(nxt,math.inf):
                    cost[nxt]=ng; parent[nxt]=node
                    heapq.heappush(queue,(ng+math.dist(pos(nxt),goal),ng,nxt))
        self.cache[key]=None
        return None

    def distance(self, a, b):
        p=self.path(a,b)
        return math.inf if p is None else sum(math.dist(x,y) for x,y in zip(p,p[1:]))


def route_cost(points, home, planner):
    locations=[tuple(home)]+[p.xy for p in points]+[tuple(home)]
    return sum(planner.distance(a,b) for a,b in zip(locations,locations[1:]))


def route(points, home, planner, method='2opt'):
    if method not in ('original','nearest','2opt'):
        raise ValueError('route method must be original, nearest or 2opt')
    # Priority 1 is first. Original order is retained within each priority group.
    groups={k:[p for p in points if p.priority==k] for k in sorted({p.priority for p in points})}
    result=[]; current=tuple(home)
    for group in groups.values():
        if method=='original':
            ordered=group[:]
        else:
            remaining=group[:]; ordered=[]
            while remaining:
                chosen=min(remaining,key=lambda p:(planner.distance(current,p.xy),p.point_id))
                remaining.remove(chosen); ordered.append(chosen); current=chosen.xy
            if method=='2opt':
                improved=True
                while improved:
                    improved=False
                    for i in range(len(ordered)-1):
                        for j in range(i+1,len(ordered)):
                            candidate=ordered[:i]+list(reversed(ordered[i:j+1]))+ordered[j+1:]
                            before=route_cost(result+ordered,home,planner)
                            after=route_cost(result+candidate,home,planner)
                            if after+1e-8<before:
                                ordered=candidate; improved=True
        result.extend(ordered)
        if ordered: current=ordered[-1].xy
    return result


class MarkStore:
    def __init__(self):
        self.attempts={}

    def deposit(self, attempt_id, marker_xy, ground_z, rng, failure_probability=0, sigma=0):
        if attempt_id not in self.attempts:
            if rng.random()<failure_probability:
                result={'ack': True,'deposited': False,'reason':'injected deposition failure'}
            else:
                result={'ack': True,'deposited': True,'actual': [marker_xy[0]+rng.gauss(0,sigma),marker_xy[1]+rng.gauss(0,sigma),ground_z], 'reason':'deposited'}
            self.attempts[attempt_id]=result
        return self.attempts[attempt_id]


def verify(target, result, tolerance):
    if not result or not result.get('ack'):
        return False, None, 'missing acknowledgement'
    if not result.get('deposited'):
        return False, None, 'no deposition'
    error=math.hypot(result['actual'][0]-target.x,result['actual'][1]-target.y)
    return error<=tolerance,error,'verified' if error<=tolerance else 'outside tolerance'


def run_test_backend(scenario_path, method='2opt', seed=None, output_root='reports'):
    """Fast deterministic test run. Positions are idealized, never Isaac physics."""
    scenario_path=Path(scenario_path); cfg=load_config(scenario_path)
    if cfg.get('localization') or 'navigation_timeout_s' in cfg:
        raise NotImplementedError('deterministic backend cannot evaluate localization faults or navigation timeouts')
    if seed is not None: cfg['seed']=seed
    layout=Path(cfg['layout'])
    if not layout.is_absolute(): layout=scenario_path.parents[2]/layout
    points,rejected=load_layout(layout,cfg['bounds'])
    planner=GridPlanner(cfg['bounds'],cfg.get('obstacles',[]),cfg['robot_radius'],cfg['clearance'],cfg['grid_resolution'])
    home=tuple(cfg['home'])
    if not planner.free(home):
        raise ValueError('home conflicts with inflated obstacle or boundary clearance')
    blocked=[{'point_id':p.point_id,'reason':'inside inflated obstacle or unreachable'} for p in points if planner.path(home,p.xy) is None]
    blocked_ids={x['point_id'] for x in blocked}
    candidates=[p for p in points if p.point_id not in blocked_ids]
    ordered=route(candidates,home,planner,method)
    run_hash=hashlib.sha256(json.dumps({'cfg':cfg,'method':method},sort_keys=True).encode()).hexdigest()[:12]
    stem=scenario_path.stem+'_'+method+'_'+str(cfg['seed'])+'_'+run_hash
    root=Path(output_root); root.mkdir(parents=True,exist_ok=True)
    for serial in range(1,10000):
        out=root/f'{stem}_{serial:04d}'
        try:
            out.mkdir(exist_ok=False)
            break
        except FileExistsError:
            continue
    else:
        raise RuntimeError('too many report directories with this configuration')
    deposition_rng=random.Random(cfg['seed']+10001)
    failure_rng=random.Random(cfg['seed']+20003)
    store=MarkStore(); records=[]; t=0.0; location=home
    for p in ordered:
        distance=planner.distance(location,p.xy)
        t+=distance/cfg['speed']
        attempts=[]; success=False
        for index in range(cfg['max_retries']+1):
            attempt_id=f'{p.point_id}-{index+1}'
            fail=failure_rng.random()<cfg.get('failure_probability',0)
            mark=store.deposit(attempt_id,p.xy,0,deposition_rng,1 if fail else 0,cfg.get('deposition_sigma',0))
            ok,error,reason=verify(p,mark,cfg['mark_tolerance'])
            attempts.append({'attempt_id':attempt_id,'time_s':t,'result':mark,'error_m':error,'status':reason})
            t+=cfg.get('stabilization_s',1.5)+cfg.get('deposition_s',0.3)
            if ok: success=True; break
        records.append({'point_id':p.point_id,'target':asdict(p),'status':'completed' if success else 'failed','attempts':attempts})
        location=p.xy
    t+=planner.distance(location,home)/cfg['speed']
    errors=[a['error_m'] for r in records for a in r['attempts'] if a['error_m'] is not None and a['status']=='verified']
    errors.sort(); n=len(errors)
    metrics={'mean_m':sum(errors)/n if n else None,'rmse_m':math.sqrt(sum(e*e for e in errors)/n) if n else None,
             'max_m':max(errors) if n else None,'p95_m':errors[math.ceil(.95*n)-1] if n else None}
    manifest_path=scenario_path.parents[2]/'assets/generated/asset_manifest.json'
    asset_hashes=json.loads(manifest_path.read_text()).get('source_sha256',{}) if manifest_path.exists() else {}
    result={'backend':'deterministic_test_backend','scenario':scenario_path.stem,'seed':cfg['seed'],'route_method':method,
            'asset_sha256':asset_hashes,'software':{'python':sys.version.split()[0]},
            'configuration':cfg,'configuration_sha256':hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest(),
            'planned_route_m':route_cost(ordered,home,planner),'simulated_duration_s':t,
            'completed':sum(r['status']=='completed' for r in records),'failed':sum(r['status']=='failed' for r in records),
            'blocked':blocked,'rejected':rejected,'records':records,'metrics':metrics,'returned_home':True,'landed':True,
            'success_rate_denominator':'valid unblocked targets','success_rate':sum(r['status']=='completed' for r in records)/len(ordered) if ordered else None,
            'terminal_reason':'complete' if not blocked and not rejected and all(r['status']=='completed' for r in records) else 'partial_failure'}
    (out/'report.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    with (out/'targets.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=['point_id','status','attempts','last_error_m'])
        writer.writeheader()
        for r in records:
            writer.writerow({'point_id':r['point_id'],'status':r['status'],'attempts':len(r['attempts']),'last_error_m':r['attempts'][-1]['error_m']})
        for b in blocked: writer.writerow({'point_id':b['point_id'],'status':'blocked','attempts':0,'last_error_m':''})
    return out,result
