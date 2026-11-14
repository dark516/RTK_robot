from __future__ import annotations
from dataclasses import asdict
from enum import IntEnum
import json

from frob_city_navigation.mission import (
    MissionMachine, RunContext, MissionEvent, MissionEventKind, PlanStatus,
    ValidationStatus, ServiceReceipt, MissionPhase,
)
from frob_city_navigation.models import Pose2D

BUS_A='BUS_STOP:A:FACE';BUS_B='BUS_STOP:B:FACE';PARK='PARKING:P:FACE'

def event(m,kind,**kw):
    s=m.state();kw.setdefault('run_id',s.run_id);kw.setdefault('map_instance_id',s.map_instance_id);kw.setdefault('navigator_instance_id',s.navigator_instance_id);return MissionEvent(kind=kind,**kw)

def trace_once():
    m=MissionMachine(); trace=[]
    def add(label,tr):trace.append((label,tr.accepted,int(tr.snapshot.phase),int(tr.snapshot.execution_state),tuple((int(c.kind),c.target_id,c.route_id,c.segment_id,c.goal_generation) for c in tr.commands)))
    add('start',m.start(RunContext('run','map','nav',2,False,True,'',(BUS_A,BUS_B),PARK,0)))
    seq=0
    for i,target in enumerate((BUS_A,BUS_B,PARK),1):
        s=m.state();assert s.objective_id==target
        route=f'R{i}';seg=f'S{i}'
        add('plan',m.handle(event(m,MissionEventKind.PLAN_RESULT,request_id=s.current_request_id,map_revision=i,route_id=route,segment_id=seg,plan_status=PlanStatus.ROUTE_READY)))
        add('valid',m.handle(event(m,MissionEventKind.SEGMENT_VALIDATION_RESULT,route_id=route,segment_id=seg,validation_status=ValidationStatus.VALID,map_revision=i)))
        gen=m.state().goal_generation
        add('accept',m.handle(event(m,MissionEventKind.GOAL_ACCEPTED,goal_generation=gen,route_id=route,segment_id=seg)))
        seq+=1;add('done',m.handle(event(m,MissionEventKind.EXECUTION_COMPLETED,goal_generation=gen,route_id=route,segment_id=seg,report_sequence=seq,reached_objective=True)))
        if i==1:
            old=m.handle(event(m,MissionEventKind.EXECUTION_COMPLETED,goal_generation=0,route_id=route,segment_id=seg,report_sequence=99,reached_objective=True));assert not old.accepted;add('old',old)
        if target!=PARK:
            receipt=ServiceReceipt('run',target,i*10.,i*10.+2.2,2.2,Pose2D(float(i),0.,0.),0.,i)
            add('service',m.handle(event(m,MissionEventKind.SERVICE_COMPLETED,target_id=target,receipt=receipt)))
        else:add('park',m.handle(event(m,MissionEventKind.PARKING_STOP_VERIFIED,target_id=PARK)))
    assert m.state().phase is MissionPhase.DONE
    assert m.state().serviced_stop_ids==(BUS_A,BUS_B)
    return trace

def test_wp14_deterministic_trace_is_identical_repeatedly():
    first=trace_once(); assert all(trace_once()==first for _ in range(9))

def test_wp14_parking_cannot_replace_two_unique_receipts():
    m=MissionMachine();m.start(RunContext('run','map','nav',2,False,True,'',(BUS_A,BUS_B),PARK,0))
    premature=m.handle(event(m,MissionEventKind.PARKING_STOP_VERIFIED,target_id=PARK))
    assert not premature.accepted
    assert m.state().phase is not MissionPhase.DONE
