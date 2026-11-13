"""Explicitly defined metrics, no implicit claims about safety or optimality."""
from math import sqrt
from .simulation import Sample


def summarize(rows: list[Sample], dt_s: float) -> dict[str,float]:
    if not rows or dt_s <= 0:
        raise ValueError('nonempty rows and positive dt required')
    n=len(rows)
    target=rows[0].target_m_s
    errors=[s.speed_m_s-s.target_m_s for s in rows]
    return {
        'speed_rmse_m_s':sqrt(sum(e*e for e in errors)/n),
        'speed_mae_m_s':sum(abs(e) for e in errors)/n,
        'max_overspeed_m_s':max(errors),
        'final_speed_error_m_s':errors[-1],
        'distance_km':rows[-1].position_m/1000,
        'drive_energy_kwh':rows[-1].cumulative_drive_kwh,
        'recovered_energy_kwh':rows[-1].cumulative_regen_kwh,
        'net_battery_energy_kwh':rows[-1].cumulative_drive_kwh-rows[-1].cumulative_regen_kwh,
        'rms_jerk_m_s3':sqrt(sum(s.jerk_m_s3**2 for s in rows)/n),
        'peak_abs_acceleration_m_s2':max(abs(s.acceleration_m_s2) for s in rows),
        'saturation_fraction':sum(abs(s.requested_force_n-s.applied_force_n)>1e-8 for s in rows)/n,
        'friction_braking_kwh':sum(s.friction_brake_force_n*s.speed_m_s*dt_s/3_600_000 for s in rows),
    }
