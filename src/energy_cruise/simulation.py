"""Deterministic fixed-step simulation with explicit sample-and-hold control."""
from dataclasses import dataclass, replace
from math import isfinite
from .model import Vehicle,Environment,road_grade,road_grade_preview,longitudinal_forces,apply_actuator
from .controllers import PIController,PreviewController


@dataclass(frozen=True)
class Scenario:
    duration_s: float = 150.0
    dt_s: float = 0.05
    initial_speed_m_s: float = 100/3.6
    target_speed_m_s: float = 100/3.6
    lookahead_m: float = 100.0
    preview_bias: float = 0.0
    mass_scale: float = 1.0
    headwind_m_s: float = 0.0

    def __post_init__(self):
        for name in ('duration_s','dt_s','target_speed_m_s','initial_speed_m_s','lookahead_m','mass_scale','preview_bias','headwind_m_s'):
            if not isfinite(getattr(self,name)):
                raise ValueError(f'{name} must be finite')
        if self.duration_s <= 0 or self.dt_s <= 0 or self.dt_s > self.duration_s or self.mass_scale <= 0:
            raise ValueError('duration/dt/mass_scale must be positive and dt <= duration')
        if self.initial_speed_m_s < 0 or self.target_speed_m_s < 0 or self.lookahead_m < 0:
            raise ValueError('speeds and lookahead must be nonnegative')
        if self.duration_s/self.dt_s > 1_000_000:
            raise ValueError('too many simulation steps')


@dataclass(frozen=True)
class Sample:
    time_s: float
    position_m: float
    speed_m_s: float
    target_m_s: float
    grade: float
    preview_grade: float
    requested_force_n: float
    applied_force_n: float
    regen_force_n: float
    friction_brake_force_n: float
    acceleration_m_s2: float
    battery_kwh: float
    cumulative_drive_kwh: float
    cumulative_regen_kwh: float
    jerk_m_s3: float


def run_simulation(controller: PIController, scenario: Scenario = Scenario(), vehicle: Vehicle = Vehicle(), env: Environment = Environment()) -> list[Sample]:
    """Forward Euler, fixed step; final partial interval is supported.

    Initial position is zero. No hidden future truth is supplied to controller.
    Battery energy is a *simplified* wheel-to-battery accounting model, not a calibrated EV powertrain.
    """
    vehicle=replace(vehicle,mass_kg=vehicle.mass_kg*scenario.mass_scale)
    env=replace(env,wind_m_s=scenario.headwind_m_s)
    controller.reset()
    t=0.0; x=0.0; v=scenario.initial_speed_m_s
    battery=0.8*vehicle.battery_capacity_kwh
    drive_energy=0.0; regen_energy=0.0; last_acc=0.0
    rows=[]
    while t < scenario.duration_s - 1e-10:
        dt=min(scenario.dt_s,scenario.duration_s-t)
        grade=road_grade(x)
        preview=road_grade_preview(x,scenario.lookahead_m,scenario.preview_bias)
        if isinstance(controller,PreviewController):
            requested=controller.command_with_current_grade(scenario.target_speed_m_s,v,grade,preview,vehicle,env)
        else:
            requested=controller.command(scenario.target_speed_m_s,v,grade,vehicle,env)
        applied,regen,friction=apply_actuator(requested,v,vehicle)
        aero,rolling,hill=longitudinal_forces(v,grade,vehicle,env)
        net=applied-aero-rolling-hill
        # A standstill vehicle cannot accelerate backwards in this forward-only model.
        acc=max(0.0,net/vehicle.mass_kg) if v <= 0 and net < 0 else net/vehicle.mass_kg
        v_next=max(0.0,v+dt*acc)
        # Drive energy from wheels / drivetrain efficiency; regen returned to battery.
        drive_w=max(applied,0.0)*v/vehicle.drivetrain_efficiency
        regen_w=regen*v*vehicle.regen_efficiency
        delta_drive=drive_w*dt/3_600_000
        delta_regen=regen_w*dt/3_600_000
        drive_energy+=delta_drive; regen_energy+=delta_regen
        battery=max(0.0,min(vehicle.battery_capacity_kwh,battery-delta_drive+delta_regen))
        jerk=(acc-last_acc)/dt if rows else 0.0
        rows.append(Sample(t,x,v,scenario.target_speed_m_s,grade,preview,requested,applied,regen,friction,acc,battery,drive_energy,regen_energy,jerk))
        controller.update(scenario.target_speed_m_s-v,requested,applied,dt)
        x+=0.5*(v+v_next)*dt
        v=v_next; t+=dt; last_acc=acc
    return rows
