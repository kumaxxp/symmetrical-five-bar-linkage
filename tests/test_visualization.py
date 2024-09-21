import unittest
from src.kinematics.hip import Hip
from src.visualization.visualization import Visualization

class TestVisualization(unittest.TestCase):
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

        self.hip.left_leg.Rfe = 123  # テスト用の値を設定


if __name__ == '__main__':
    unittest.main()

"""

# プロジェクトのルートディレクトリから実行
python -m unittest discover -s tests

# tests ディレクトリに移動してから実行
cd tests
python -m unittest discover

"""