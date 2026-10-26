# SPDX-License-Identifier: Apache-2.0
import pathlib,sys,unittest,math
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
from benchmark import physics_rows,solve4,ordinary_least_squares,generate
class TestBenchmark(unittest.TestCase):
    def test_generator_physics_dimensions(self):
        m,c,rows=generate('compact_arm','clean',11)
        self.assertEqual(len(rows),80)
        self.assertEqual(len(rows[0]),13)
        self.assertTrue(all(math.isfinite(x) for row in rows for x in row))
    def test_ols_clean_is_valid_baseline(self):
        m,c,rows=generate('extended_arm','clean',12)
        o=ordinary_least_squares(rows)
        self.assertLess(abs(m-o['mass_kg']),.05)
        self.assertLess(math.dist(c,o['com_sensor_m']),.02)
    def test_rank_deficient_solver_abstains(self):
        self.assertIsNone(solve4([[0.0]*4 for _ in range(4)],[0]*4))
    def test_single_direction_3d_ols_abstains(self):
        m,c,rows=generate('extended_arm','clean',12)
        for r in rows:r[1:4]=[0,0,-9.80665]
        # Force/torque perturbed measurements mean the inconsistent data do not
        # restore rank merely by containing noise.
        self.assertIsNone(ordinary_least_squares(rows))
if __name__=='__main__':unittest.main()
