"""Offline ACC research baseline, not a safety-certified controller."""
from dataclasses import dataclass
from math import isfinite

@dataclass(frozen=True)
class Config:
    target: float = 27.78
    headway: float = 1.8
    min_gap: float = 5.0
    comfort_decel: float = 2.0
    max_accel: float = 1.5
    max_jerk: float = 2.0
    emergency_decel: float = 6.0
    dt: float = 0.05

@dataclass(frozen=True)
class State:
    ego_x: float
    ego_v: float
    lead_x: float
    lead_v: float
    previous_accel: float = 0.0

def command(s: State, c: Config):
    if not all(isfinite(x) for x in vars(s).values()) or min(s.ego_v,s.lead_v)<0:
        raise ValueError("invalid state")
    gap=s.lead_x-s.ego_x
    closing=max(0.0,s.ego_v-s.lead_v)
    if gap <= c.min_gap + closing*closing/(2*c.emergency_decel)+closing*c.dt:
        return -c.emergency_decel, True
    desired=c.min_gap+c.headway*s.ego_v
    raw=min(0.5*(c.target-s.ego_v),0.18*(gap-desired)-0.9*(s.ego_v-s.lead_v))
    raw=max(-c.comfort_decel,min(c.max_accel,raw))
    a=max(s.previous_accel-c.max_jerk*c.dt,min(s.previous_accel+c.max_jerk*c.dt,raw))
    return a,False

def simulate(duration=30.0, initial_gap=70.0, lead_speed=20.0, config=None):
    c=config or Config()
    if duration<=0 or initial_gap<=0 or lead_speed<0 or c.dt<=0 or c.emergency_decel<=0:
        raise ValueError("invalid scenario")
    s=State(0.0,c.target,initial_gap,lead_speed)
    rows=[]
    n=int(duration/c.dt)
    if n>1000000: raise ValueError("too many steps")
    for i in range(n):
        a,emergency=command(s,c)
        rows.append(dict(time_s=i*c.dt,gap_m=s.lead_x-s.ego_x,ego_speed_m_s=s.ego_v,lead_speed_m_s=s.lead_v,accel_m_s2=a,emergency=int(emergency)))
        v=max(0.0,s.ego_v+a*c.dt)
        s=State(s.ego_x+0.5*(s.ego_v+v)*c.dt,v,s.lead_x+s.lead_v*c.dt,s.lead_v,a)
    return rows
