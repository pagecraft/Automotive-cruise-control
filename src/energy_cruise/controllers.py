"""Controllers with identical interfaces. Force units are newtons."""
from dataclasses import dataclass
from math import isfinite
from .model import Vehicle, Environment, longitudinal_forces


@dataclass
class PIController:
    kp: float = 850.0
    ki: float = 55.0
    kaw: float = 0.5
    integral_n: float = 0.0

    def __post_init__(self):
        if any(not isfinite(x) or x < 0 for x in (self.kp,self.ki,self.kaw)):
            raise ValueError('controller gains must be finite and nonnegative')

    def reset(self):
        self.integral_n=0.0

    def command(self, target_m_s: float, speed_m_s: float, grade_estimate: float, vehicle: Vehicle, env: Environment) -> float:
        return self.kp*(target_m_s-speed_m_s)+self.integral_n

    def update(self, error_m_s: float, raw_n: float, applied_n: float, dt_s: float) -> None:
        if dt_s <= 0 or not isfinite(dt_s):
            raise ValueError('dt_s must be finite and positive')
        self.integral_n += dt_s*(self.ki*error_m_s+self.kaw*(applied_n-raw_n))


@dataclass
class PreviewController(PIController):
    """PI + resistance feedforward + early *bounded* grade-transition preparation.

    No future vehicle state or future true grade is available to this controller.
    The preview is a single noisy/biasable measurement at a fixed lookahead.
    """
    preview_gain: float = 0.35

    def __post_init__(self):
        super().__post_init__()
        if not isfinite(self.preview_gain) or not 0 <= self.preview_gain <= 1:
            raise ValueError('preview_gain must be in [0,1]')

    def command(self, target_m_s: float, speed_m_s: float, grade_estimate: float, vehicle: Vehicle, env: Environment) -> float:
        # Current grade is measured from an onboard inclinometer, supplied separately.
        raise RuntimeError('Use command_with_current_grade for preview control')

    def command_with_current_grade(self, target_m_s: float, speed_m_s: float, current_grade: float, preview_grade: float, vehicle: Vehicle, env: Environment) -> float:
        aero,rolling,hill=longitudinal_forces(target_m_s,current_grade,vehicle,env)
        # Blend future grade only in the feedforward path, not as the current plant grade.
        future_hill=longitudinal_forces(target_m_s,preview_grade,vehicle,env)[2]
        ff=aero+rolling+hill+self.preview_gain*(future_hill-hill)
        return super().command(target_m_s,speed_m_s,current_grade,vehicle,env)+ff
