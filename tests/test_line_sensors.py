import unittest
from ondroid_ros2.line_sensors import analyse, simulated
from ondroid_ros2.simulation import Simulation

class Sensors(unittest.TestCase):
    def test_channel_order_and_absence(self):
        self.assertEqual(analyse([960]+[100]*7)['error'],-1)
        self.assertEqual(analyse([100]*7+[960])['error'],1)
        self.assertEqual(analyse([100]*3+[960]*2+[100]*3)['error'],0)
        self.assertFalse(analyse([100]*8)['line_visible'])
        self.assertIsNone(analyse([100]*8)['error'])
        self.assertFalse(analyse([100,101,102,103,104,103,102,101])['line_visible'])
    def test_reject_bad_data(self):
        for raw in ([1]*7,[True]*8,[4096]*8,[-1]*8,[float('nan')]*8):
            with self.assertRaises(ValueError):analyse(raw)
    def test_mask_and_closed_loop_finish(self):
        reading=simulated(.4,-.185,0)
        self.assertEqual(reading['raw'],[100,100,100,100,960,960,100,100])
        clock=[0.];sim=Simulation(clock=lambda:clock[0]);seen=0
        for _ in range(200):
            p=sim.sample();r=p['line'];seen+=r['line_visible']
            if ((p['x']-.6)**2+(p['y']+.155)**2)**.5<=.028:break
            sim.drive(.025 if r['error'] is not None else 0,.6*(r['error'] or 0),None);clock[0]+=.1
        self.assertLess(((p['x']-.6)**2+(p['y']+.155)**2)**.5,.028)
        self.assertGreater(seen,40)
        sim.stop()

if __name__=='__main__':unittest.main()
