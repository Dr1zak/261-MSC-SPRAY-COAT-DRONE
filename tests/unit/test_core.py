import csv
import math
import random
from pathlib import Path

import pytest

from construction_autonomy.core import (GridPlanner, MarkStore, Point, load_layout,
                                         route, route_cost, verify, run_test_backend)


def planner(obstacles=()):
    return GridPlanner([0,10,0,10],obstacles,0.2,0.2,0.5)


def test_layout_rejects_invalid_rows(tmp_path):
    path=tmp_path/'points.csv'
    path.write_text('point_id,x_m,y_m,z_m,mark_type,priority\nA,1,1,0,dot,1\nA,2,2,0,dot,1\nB,nan,2,0,dot,1\nC,11,2,0,dot,1\nD,3,3,1,dot,1\nE,3,3,0,line,1\nF,3,3,0,dot,0\n')
    valid,rejected=load_layout(path,[0,10,0,10])
    assert [p.point_id for p in valid]==['A']
    assert len(rejected)==6


def test_collision_inflation_and_segment():
    p=planner([[4,6,4,6]])
    assert not p.free((3.8,5))
    assert not p.segment_free((1,5),(9,5))
    route=p.path((1,5),(9,5))
    assert route and all(p.segment_free(a,b) for a,b in zip(route,route[1:]))
    assert p.distance((1,5),(9,5))>8


def test_unreachable():
    p=planner([[4,6,0,10]])
    assert p.path((1,5),(9,5)) is None


def test_routing_deterministic_and_home_inclusive():
    p=planner()
    points=[Point('B',8,1,0,'dot',1),Point('A',2,1,0,'dot',1),Point('C',4,1,0,'dot',1)]
    ordered=route(points,(1,1),p,'nearest')
    assert [x.point_id for x in ordered]==['A','C','B']
    assert math.isclose(route_cost(ordered,(1,1),p),14)
    assert route_cost(route(points,(1,1),p,'2opt'),(1,1),p)<=route_cost(ordered,(1,1),p)


def test_mark_idempotent_and_threshold():
    store=MarkStore(); rng=random.Random(3)
    a=store.deposit('mission-A-1',(1,1),0,rng,sigma=.01)
    b=store.deposit('mission-A-1',(9,9),0,rng,sigma=2)
    assert a is b and len(store.attempts)==1
    point=Point('A',1,1,0,'dot',1)
    assert verify(point,{'ack':True,'deposited':True,'actual':[1.05,1,0]},.1)[0]
    assert not verify(point,{'ack':True,'deposited':False},.1)[0]


def test_baseline_report(tmp_path):
    out,result=run_test_backend('config/scenarios/baseline.yaml',output_root=tmp_path)
    assert result['backend']=='deterministic_test_backend'
    assert result['completed']==50 and result['failed']==0
    assert result['planned_route_m']>0 and result['returned_home']
    assert (out/'report.json').exists() and len(list(csv.DictReader((out/'targets.csv').open())))==50


def test_obstacles_classify_blocked(tmp_path):
    _,result=run_test_backend('config/scenarios/obstacles.yaml','nearest',output_root=tmp_path)
    assert result['blocked'] and result['terminal_reason']=='partial_failure'
