import itertools
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from physics import Vehicle, transition, battery_power, steady_optimum, G, RHO
from model import routes, route, optimize, evaluate, preset, compare, graph, TIME_QUANTUM
from app import dispatch


class PhysicsTests(unittest.TestCase):
    def test_constant_speed_analytic_integral(self):
        car=Vehicle()
        e=transition(100,0,10,10,car)
        force=car.mass*G*car.crr+.5*RHO*car.cd*car.area*100
        expected=(force*100/car.efficiency+car.auxiliary*10)/3600
        self.assertAlmostEqual(e['energy'],expected,places=10)

    def test_inertia_integral_equals_kinetic_energy(self):
        car=Vehicle()
        e=transition(100,0,4,12,car)
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e['inertia'],.5*car.mass*(12**2-4**2)/3600,places=10)

    def test_gravity_work_and_angle_use_arc_length(self):
        car=Vehicle()
        e=transition(100,10,8,8,car)
        self.assertAlmostEqual(e['grade'],car.mass*G*10/3600,places=10)
        self.assertAlmostEqual(e['grade_angle'],math.degrees(math.asin(.1)))

    def test_regen_is_capped_and_idle_draws_power(self):
        car=Vehicle()
        self.assertEqual(battery_power(-100000,car),-car.regen_power*car.regen+car.auxiliary)
        self.assertEqual(battery_power(0,car),car.auxiliary)
        self.assertEqual(battery_power(-100000,Vehicle(regen=0)),car.auxiliary)

    def test_regenerative_round_trip_cannot_create_energy(self):
        car=Vehicle()
        up=transition(100,10,8,8,car)
        down=transition(100,-10,8,8,car)
        self.assertLess(down['energy'],0)
        self.assertGreater(up['energy']+down['energy'],0)

    def test_force_and_power_constraints(self):
        self.assertIsNone(transition(25,0,0,20,Vehicle()))
        self.assertIsNone(transition(25,0,10,12,Vehicle(),limit=11))
        self.assertIsNone(transition(25,9,16,16,Vehicle()))
        self.assertIsNone(transition(25,0,0,0,Vehicle()))

    def test_optimal_flat_speed_is_stationary_minimum(self):
        car=Vehicle()
        v=steady_optimum(car)
        def cost(speed):
            return (car.crr*car.mass*G+.5*RHO*car.cd*car.area*speed**2)/car.efficiency+car.auxiliary/speed
        self.assertLess(cost(v),cost(v-.2))
        self.assertLess(cost(v),cost(v+.2))
        self.assertAlmostEqual((cost(v+.0001)-cost(v-.0001))/.0002,0,places=6)
        self.assertEqual(steady_optimum(Vehicle(auxiliary=0)),0)

    def test_headwind_increases_consumption(self):
        still=transition(100,0,10,10,Vehicle())
        windy=transition(100,0,10,10,Vehicle(wind=5))
        self.assertGreater(windy['energy'],still['energy'])


class RouteTests(unittest.TestCase):
    def test_candidates_share_elevation_and_checkpoint(self):
        for r in routes():
            self.assertEqual((r['h'][0],r['h'][-1]),(0,0))
            self.assertEqual(r['h'][r['stops'][0]],8)

    def test_optimized_routes_meet_constraints_and_energy_balance(self):
        for r in routes():
            with self.subTest(route=r['id']):
                result=optimize(r,Vehicle(),220)
                self.assertIsNotNone(result)
                self.assertTrue(result['valid'])
                self.assertLessEqual(result['time'],result['conservative_time']+1e-8)
                self.assertLessEqual(result['conservative_time'],220)
                self.assertEqual(result['speeds'][r['stops'][0]],0)
                totals=result['totals']
                self.assertAlmostEqual(result['energy'],totals['draw']+totals['auxiliary']-totals['recovered'],places=9)
                self.assertAlmostEqual(totals['grade'],0,places=9)
                self.assertAlmostEqual(totals['inertia'],0,places=9)
                self.assertEqual(result['samples'][-1]['s'],r['length'])
                idle=[(a,b) for a,b in zip(result['samples'],result['samples'][1:]) if a['s']==b['s']]
                self.assertEqual(len(idle),1)
                self.assertAlmostEqual(idle[0][1]['t']-idle[0][0]['t'],4)

    def test_dp_matches_exhaustive_enumeration_on_small_graph(self):
        r={'id':'test','label':'test','s':[0,25,50,75,100],'h':[0,1,2,1,0],
           'limits':[10]*4,'stops':[],'length':100}
        car=Vehicle()
        result=optimize(r,car,30)
        possible=[]
        for middle in itertools.product([2.,4.,6.,8.,10.],repeat=3):
            speeds=[0.,*middle,0.]
            edges=[transition(25,r['h'][i+1]-r['h'][i],speeds[i],speeds[i+1],car,10) for i in range(4)]
            if all(e is not None for e in edges) and sum(math.ceil(e['dt']/TIME_QUANTUM-1e-9) for e in edges)*TIME_QUANTUM<=30:
                possible.append(sum(e['energy'] for e in edges))
        self.assertAlmostEqual(result['energy'],min(possible),places=10)

    def test_infeasible_time_returns_no_solution(self):
        self.assertIsNone(optimize(routes()[0],Vehicle(),1))

    def test_late_user_not_awarded_savings(self):
        r=routes()[0]
        user=preset(r,Vehicle(),'wave',80)
        self.assertFalse(user['valid'])
        result=compare(user,[optimize(r,Vehicle(),220)])
        self.assertIsNone(result['saving'])

    def test_presets_and_manual_graph_can_finish(self):
        for r in routes({'length':800,'peak1':50,'peak2':-20,'limit':30}):
            for style in ('wave','smooth','fast'):
                p=preset(r,Vehicle(),'wave' if style=='wave' else style,480)
                self.assertTrue(p['valid'])

    def test_invalid_stops_not_fair(self):
        r={'id':'test','label':'test','s':[0,100,200],'h':[0,0,0],
           'limits':[20,20],'stops':[1],'length':200}
        result=evaluate(r,[0,10,0],Vehicle(),220)
        self.assertFalse(result['valid'])
        self.assertIn('필수 정차 누락',result['reasons'])

    def test_api_and_nonfinite_inputs(self):
        self.assertEqual(len(dispatch('/api/config',{})['routes']),3)
        for bad in [float('nan'),float('inf'),-1]:
            with self.assertRaises(ValueError):
                dispatch('/api/config',{'vehicle':{'mass':bad}})
        with self.assertRaises(ValueError):
            dispatch('/api/config',{'custom':{'length':850}})
        with self.assertRaises(ValueError):
            dispatch('/api/compare',{'speeds':[]})
        with self.assertRaises(ValueError):
            dispatch('/api/step',{'index':0,'v':0,'target':float('nan')})
        with self.assertRaises(ValueError):
            dispatch('/api/config',{'vehicle':[]})
        with self.assertRaises(ValueError):
            dispatch('/api/config',{'custom':[]})


if __name__=='__main__':
    unittest.main()
