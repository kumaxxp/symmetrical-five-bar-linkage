import numpy as np
import math
from kinematics.extended_kinematics import ExtendedKinematics
from kinematics.transformation import Transformation2D

class Hip:
    def __init__(self):
        self.left_leg = None
        self.right_leg = None

    def setup(self, B1, B2):
        self.left_leg = ExtendedKinematics(150, 200, 150, 150, B1, B2)
        self.right_leg = ExtendedKinematics(150, 200, 150, 150, B1, B2)

    def set_leg_angles(self, leg, theta1, theta2, thetaF):
        if leg not in ['left', 'right']:
            raise ValueError("Invalid leg name. Must be 'left' or 'right'.")

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

    B1 = (50, -100 + 200)
    B2 = (-50, -100 + 200)
    hip.setup(B1, B2)

    initial_angles = {
        'left': {'theta1': -50, 'theta2': -120, 'thetaF': -60},
        'right': {'theta1': -50, 'theta2': -120, 'thetaF': -60}
    }

    hip.set_leg_angles('left', *initial_angles['left'])
    hip.set_leg_angles('right', *initial_angles['right'])

    hip.compute_forward_kinematics()

    """
    # 左脚の回転（PointE中心）
    points = self.left_leg.get_original_points()
    self.left_leg.calculate_rotated_points(points['E'], points['F'], (0, 0), (1, 0))

    # 左脚の回転後のポイント(B1,B2)
    rotate_points = self.left_leg.get_rotated_points()
    
    # 右脚の回転（B1,B2中心）。B1,B2に向かって平行移動
    points = self.right_leg.get_original_points()
    self.right_leg.calculate_rotated_points( points['B2'], points['B1'], rotate_points['B2'], rotate_points['B1'])
    self.right_leg.calculate_translate_points( rotate_points['B2'])
    
    @staticmethod
    def _angle_between_vectors(v1_start, v1_end, v2_start, v2_end):
        v1 = np.array(v1_end) - np.array(v1_start)
        v2 = np.array(v2_end) - np.array(v2_start)
        angle_rad = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
        angle_deg = np.degrees(angle_rad)
        return angle_deg if -180 <= angle_deg <= 180 else angle_deg - 360 * np.sign(angle_deg)
    """
