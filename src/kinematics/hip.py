import numpy as np
import math
from kinematics.extended_kinematics import ExtendedKinematics
from kinematics.transformation import Transformation2D

class Hip:
    def __init__(self):
        self.left_leg = None
        self.right_leg = None

    def setup(self, B1, B2):
        self.left_leg = ExtendedKinematics(350, 400, 300, 300, B1, B2)
        self.right_leg = ExtendedKinematics(350, 400, 300, 300, B1, B2)

    def set_leg_angles(self, leg, theta1, theta2, thetaF):
        if leg == 'left':
            self.left_leg.set_angles(theta1, theta2, thetaF)
        elif leg == 'right':
            self.right_leg.set_angles(theta1, theta2, thetaF)

    def compute_forward_kinematics(self):
        self.left_leg.compute_forward_kinematics()
        self.right_leg.compute_forward_kinematics()

    def get_rotated_points(self):
        return {
            'left': self.left_leg.get_rotated_points(),
            'right': self.right_leg.get_rotated_points()
        }

    def compute_link_angles(self):
        self.left_leg.compute_link_angles()
        self.right_leg.compute_link_angles()

if __name__ == "__main__":
    # extended_kinematicsと同様にデフォルトのリンクの数値のメカ構造を生成
    # Y=0の地面に対して、左脚の足先E-Fが平行になるように回転させてリンクを変化させる

    hip = Hip()
    

