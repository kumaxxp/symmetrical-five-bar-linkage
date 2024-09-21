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

        B1 = (50, -100 + 200)
        B2 = (-50, -100 + 200)
        W1 = (70, -100 + 200)
        W2 = (-70, -100 + 200)

        self.hip.set_leg_param(b=150, m=200, e=150, f=150, B1=B1, B2=B2, W1=W1, W2=W2, w=20)
        self.hip.set_weights({'B1': 1, 'B2': 1})

        self.initial_angles = {
            'left': {'theta1': -50, 'theta2': -120, 'thetaF': -60},
            'right': {'theta1': -50, 'theta2': -120, 'thetaF': -60}
        }

        for leg in ['left', 'right']:
            angles = self.initial_angles[leg]
            self.hip.set_leg_angles(leg, angles['theta1'], angles['theta2'], angles['thetaF'])

    def test_set_leg_angles(self):
        self.hip.set_leg_angles('left', theta1=-45, theta2=-90, thetaF=-30)
        self.assertEqual(self.hip.left_leg.theta1, -45)
        self.assertEqual(self.hip.left_leg.theta2, -90)
        self.assertEqual(self.hip.left_leg.thetaF, -30)

        self.hip.set_leg_angles('right', theta1=-45, theta2=-90, thetaF=-30)
        self.assertEqual(self.hip.right_leg.theta1, -45)
        self.assertEqual(self.hip.right_leg.theta2, -90)
        self.assertEqual(self.hip.right_leg.thetaF, -30)

    def test_compute_forward_kinematics(self):
        self.hip.set_leg_angles('left', theta1=-45, theta2=-90, thetaF=-30)
        self.hip.set_leg_angles('right', theta1=-45, theta2=-90, thetaF=-30)
        self.hip.compute_forward_kinematics()

        rotated_points = self.hip.get_rotated_points()

        # 結果の検証（実際の値は計算結果に応じて調整してください）
        expected_B1_x = 100  # 期待値を適切に設定
        expected_B1_y = 100  # 期待値を適切に設定

        print(rotated_points)

        self.assertAlmostEqual(rotated_points['B1'][0], expected_B1_x, places=2)
        self.assertAlmostEqual(rotated_points['B1'][1], expected_B1_y, places=2)

    def test_invalid_leg(self):
        with self.assertRaises(ValueError):
            self.hip.set_leg_angles('middle', theta1=0, theta2=0, thetaF=0)

if __name__ == '__main__':
    unittest.main()