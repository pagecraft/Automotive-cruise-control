import unittest
import math
from energy_cruise.model import Vehicle,Environment,road_grade,longitudinal_forces,apply_actuator,actuator_limits

class TestPhysics(unittest.TestCase):
    def test_road_by_distance(self):
        for x,g in [(0,0),(599.9,0),(600,.06),(1399,.06),(1400,0),(1900,-.06),(2700,0)]:
            self.assertEqual(road_grade(x),g)
    def test_grade_force(self):
        v=Vehicle(); e=Environment()
        _,_,hill=longitudinal_forces(20,.06,v,e)
        self.assertAlmostEqual(hill,v.mass_kg*9.81*math.sin(math.atan(.06)))
    def test_drag_increases_quadratically(self):
        v=Vehicle();e=Environment()
        a=longitudinal_forces(10,0,v,e)[0]
        b=longitudinal_forces(20,0,v,e)[0]
        self.assertAlmostEqual(b,4*a)
    def test_power_limit(self):
        v=Vehicle()
        self.assertAlmostEqual(actuator_limits(40,v)[0],2000)
    def test_regen_and_friction(self):
        v=Vehicle()
        applied,regen,friction=apply_actuator(-4000,25,v)
        self.assertEqual(applied,-4000)
        self.assertAlmostEqual(regen,1000)
        self.assertAlmostEqual(friction,3000)
    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):Vehicle(mass_kg=0)
        with self.assertRaises(ValueError):Vehicle(regen_efficiency=1.2)
        with self.assertRaises(ValueError):road_grade(-1)
