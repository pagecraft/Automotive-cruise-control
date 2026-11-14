import unittest
from energy_cruise import Vehicle,PIController,PreviewController,Scenario,run_simulation
from energy_cruise.metrics import summarize

class TestControl(unittest.TestCase):
    def test_reproducible(self):
        scenario=Scenario(duration_s=10,dt_s=.1)
        a=run_simulation(PIController(),scenario)
        b=run_simulation(PIController(),scenario)
        self.assertEqual(a,b)
    def test_force_limits(self):
        v=Vehicle()
        rows=run_simulation(PIController(),Scenario(duration_s=100))
        self.assertTrue(all(-v.max_brake_n<=s.applied_force_n<=v.max_drive_n for s in rows))
    def test_preview_uses_biased_estimate(self):
        base=Scenario(duration_s=45,dt_s=.1)
        a=run_simulation(PreviewController(),base)
        b=run_simulation(PreviewController(),Scenario(duration_s=45,dt_s=.1,preview_bias=.03))
        self.assertNotEqual([s.applied_force_n for s in a],[s.applied_force_n for s in b])
    def test_energy_nonnegative(self):
        rows=run_simulation(PreviewController(),Scenario(duration_s=120))
        self.assertTrue(all(s.cumulative_drive_kwh>=0 and s.cumulative_regen_kwh>=0 for s in rows))
        self.assertTrue(all(s.friction_brake_force_n>=0 and s.regen_force_n>=0 for s in rows))
    def test_metrics(self):
        rows=run_simulation(PIController(),Scenario(duration_s=5))
        metrics=summarize(rows,.05)
        self.assertIn('speed_rmse_m_s',metrics)
    def test_invalid_scenario(self):
        with self.assertRaises(ValueError):Scenario(dt_s=0)
        with self.assertRaises(ValueError):Scenario(mass_scale=-1)
    def test_controller_reset(self):
        c=PIController()
        c.integral_n=100
        c.reset()
        self.assertEqual(c.integral_n,0)
