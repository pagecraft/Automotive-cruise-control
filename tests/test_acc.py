import unittest
from energy_cruise.acc import Config,State,command,simulate

class TestACC(unittest.TestCase):
    def test_emergency(self):
        self.assertEqual(command(State(0,25,6,0),Config()),(-6.0,True))
    def test_jerk_limited(self):
        a,em=command(State(0,10,500,20),Config())
        self.assertFalse(em)
        self.assertLessEqual(abs(a),0.1+1e-9)
    def test_simulation(self):
        rows=simulate()
        self.assertEqual(len(rows),600)
        self.assertTrue(all(r["ego_speed_m_s"]>=0 for r in rows))
    def test_invalid_gap(self):
        with self.assertRaises(ValueError): simulate(initial_gap=-1)
