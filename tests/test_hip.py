import unittest
import sys
import os
import math

# プロジェクトのルートディレクトリをPythonパスに追加
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from kinematics.hip import Hip

class TestHip(unittest.TestCase):
    def setUp(self):
        self.hip = Hip()
        self.hip.setup((100, 100), (-100, 100))

    def test_set_leg_angles(self):
        self.hip.set_leg_angles('left', theta1=-45, theta2=-90, thetaF=-30)
        self.assertAlmostEqual(self.hip.left_leg.theta1, math.radians(-45))
        self.assertAlmostEqual(self.hip.left_leg.theta2, math.radians(-90))
        self.assertAlmostEqual(self.hip.left_leg.thetaF, math.radians(-30))

        self.hip.set_leg_angles('right', theta1=-45, theta2=-90, thetaF=-30)
        self.assertAlmostEqual(self.hip.right_leg.theta1, math.radians(-45))
        self.assertAlmostEqual(self.hip.right_leg.theta2, math.radians(-90))
        self.assertAlmostEqual(self.hip.right_leg.thetaF, math.radians(-30))

    def test_compute_forward_kinematics(self):
        self.hip.set_leg_angles('left', theta1=-45, theta2=-90, thetaF=-30)
        self.hip.set_leg_angles('right', theta1=-45, theta2=-90, thetaF=-30)
        self.hip.compute_forward_kinematics()

        original_points = self.hip.left_leg.get_original_points()

        # 結果の検証（実際の値は計算結果に応じて調整してください）
        self.assertAlmostEqual(original_points['B1'][0], 100, places=2)
        self.assertAlmostEqual(original_points['B1'][1], 100, places=2)

    def test_invalid_leg(self):
        with self.assertRaises(ValueError):
            self.hip.set_leg_angles('middle', theta1=0, theta2=0, thetaF=0)

if __name__ == '__main__':
    unittest.main()
