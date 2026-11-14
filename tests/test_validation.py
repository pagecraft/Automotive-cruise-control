import unittest
from energy_cruise import PIController, PreviewController, Scenario, run_simulation, Vehicle
from energy_cruise.model import Environment, apply_actuator, longitudinal_forces

class TestValidation(unittest.TestCase):
    def test_zero_speed_no_negative_motion(self):
        rows=run_simulation(PIController(),Scenario(duration_s=1,initial_speed_m_s=0,target_speed_m_s=0))
        self.assertTrue(all(s.speed_m_s>=0 for s in rows))
    def test_downhill_gravity_sign(self):
        v=Vehicle();e=Environment()
        self.assertLess(longitudinal_forces(20,-.06,v,e)[2],0)
        self.assertGreater(longitudinal_forces(20,.06,v,e)[2],0)
    def test_braking_power_split(self):
        applied,regen,friction=apply_actuator(-6000,30,Vehicle())
        self.assertAlmostEqual(regen,25000/30)
        self.assertAlmostEqual(friction,6000-25000/30)
        self.assertEqual(applied,-6000)
    def test_braking_scenario(self):
        rows=run_simulation(PIController(),Scenario(duration_s=15,target_speed_m_s=60/3.6))
        self.assertLess(rows[-1].speed_m_s,rows[0].speed_m_s)
        self.assertTrue(any(s.friction_brake_force_n>0 for s in rows))
    def test_invalid_gains(self):
        with self.assertRaises(ValueError):PIController(kp=-1)
        with self.assertRaises(ValueError):PreviewController(preview_gain=2)
    def test_no_future_state_leakage(self):
        rows=run_simulation(PreviewController(),Scenario(duration_s=2))
        self.assertTrue(all(s.position_m>=0 for s in rows))
