"""Physics and grade profile; forward-only passenger-car model in SI units."""
from dataclasses import dataclass
from math import atan, cos, sin, isfinite


def _positive(name: str, value: float, zero: bool = False) -> None:
    if not isfinite(value) or (value < 0 if zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if zero else 'positive'}")


@dataclass(frozen=True)
class Vehicle:
    mass_kg: float = 1500.0
    cd: float = 0.30
    frontal_area_m2: float = 2.2
    crr: float = 0.012
    max_drive_n: float = 4500.0
    max_brake_n: float = 8000.0
    max_drive_power_w: float = 80000.0
    regen_power_w: float = 25000.0
    regen_force_n: float = 1600.0
    drivetrain_efficiency: float = 0.90
    regen_efficiency: float = 0.65
    battery_capacity_kwh: float = 55.0

    def __post_init__(self):
        for name in ('mass_kg','frontal_area_m2','max_drive_n','max_brake_n','max_drive_power_w','battery_capacity_kwh'):
            _positive(name, getattr(self,name))
        for name in ('cd','crr','regen_power_w','regen_force_n'):
            _positive(name, getattr(self,name),zero=True)
        for name in ('drivetrain_efficiency','regen_efficiency'):
            value=getattr(self,name)
            if not isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')


@dataclass(frozen=True)
class Environment:
    air_density_kg_m3: float = 1.225
    gravity_m_s2: float = 9.81
    wind_m_s: float = 0.0  # positive headwind

    def __post_init__(self):
        _positive('air_density_kg_m3',self.air_density_kg_m3)
        _positive('gravity_m_s2',self.gravity_m_s2)
        if not isfinite(self.wind_m_s):
            raise ValueError('wind_m_s must be finite')


def road_grade(position_m: float) -> float:
    """Piecewise road grade, rise/run, based on *distance* rather than time."""
    if not isfinite(position_m) or position_m < 0:
        raise ValueError('position_m must be finite and nonnegative')
    if position_m < 600: return 0.0
    if position_m < 1400: return 0.06
    if position_m < 1900: return 0.0
    if position_m < 2700: return -0.06
    return 0.0


def road_grade_preview(position_m: float, lookahead_m: float, bias: float = 0.0) -> float:
    """Imperfect grade estimate at a future position. Bias is rise/run units."""
    if not isfinite(lookahead_m) or lookahead_m < 0 or not isfinite(bias):
        raise ValueError('lookahead_m must be nonnegative and finite; bias must be finite')
    return road_grade(position_m + lookahead_m) + bias


def longitudinal_forces(speed_m_s: float, grade: float, vehicle: Vehicle, env: Environment) -> tuple[float,float,float]:
    """Return aero, rolling, grade forces in newtons (signed). Forward driving only."""
    if not isfinite(speed_m_s) or speed_m_s < 0 or not isfinite(grade):
        raise ValueError('speed must be finite/nonnegative; grade finite')
    relative = speed_m_s + env.wind_m_s
    aero = 0.5*env.air_density_kg_m3*vehicle.cd*vehicle.frontal_area_m2*relative*abs(relative)
    theta = atan(grade)
    roll = vehicle.crr*vehicle.mass_kg*env.gravity_m_s2*cos(theta)
    hill = vehicle.mass_kg*env.gravity_m_s2*sin(theta)
    return aero,roll,hill


def actuator_limits(speed_m_s: float, vehicle: Vehicle) -> tuple[float,float]:
    """Drive force is power-limited; braking includes friction + regenerative capability."""
    if not isfinite(speed_m_s) or speed_m_s < 0:
        raise ValueError('speed must be nonnegative and finite')
    drive = min(vehicle.max_drive_n, vehicle.max_drive_power_w/max(speed_m_s, 0.1))
    return drive, vehicle.max_brake_n


def apply_actuator(command_n: float, speed_m_s: float, vehicle: Vehicle) -> tuple[float,float,float]:
    """Return applied signed force, regenerative braking force, friction braking force."""
    if not isfinite(command_n):
        raise ValueError('command must be finite')
    max_drive,max_brake=actuator_limits(speed_m_s,vehicle)
    applied=max(-max_brake,min(max_drive,command_n))
    if applied >= 0: return applied,0.0,0.0
    regen=min(-applied,vehicle.regen_force_n,vehicle.regen_power_w/max(speed_m_s,0.1))
    return applied,regen,-applied-regen
