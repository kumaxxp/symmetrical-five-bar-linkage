import numpy as np
from extended_kinematics import ExtendedKinematics
from transformation import Transformation2D

class Hip:
    def __init__(self, left_leg: ExtendedKinematics, right_leg: ExtendedKinematics):
        self.left_leg = left_leg
        self.right_leg = right_leg
        self.hip_position = (0, 0)
        self.hip_angle = 0
        self.leg_distance = 200  # 左右の脚の距離（仮の値）

    def set_hip_position(self, x: float, y: float):
        self.hip_position = (x, y)

    def set_hip_angle(self, angle: float):
        self.hip_angle = angle

    def set_leg_angles(self, leg: str, theta1: float, theta2: float, thetaF: float):
        if leg == 'left':
            self.left_leg.set_angles(theta1=theta1, theta2=theta2, thetaF=thetaF)
        elif leg == 'right':
            self.right_leg.set_angles(theta1=theta1, theta2=theta2, thetaF=thetaF)
        else:
            raise ValueError("Invalid leg specified. Use 'left' or 'right'.")

    def compute_leg_positions(self):
        # 左脚の計算
        self.left_leg.compute_forward_kinematics()
        left_points = self.left_leg.get_original_points()

        # 右脚の計算
        self.right_leg.compute_forward_kinematics()
        right_points = self.right_leg.get_original_points()

        # 左脚の変換
        angle_flower_left = self._angle_between_vectors((0, 0), (1, 0), left_points['E'], left_points['F'])
        transformer_left = Transformation2D(origin=left_points['E'], angle=-angle_flower_left, translation=-left_points['E'])
        self.left_leg.apply_transformation(transformer_left)

        # 右脚の変換
        transformed_left_points = self.left_leg.get_transformed_points()
        left_b1b2_vector = np.array(transformed_left_points['B2']) - np.array(transformed_left_points['B1'])
        right_b1b2_vector = np.array(right_points['B2']) - np.array(right_points['B1'])
        
        angle = np.arctan2(left_b1b2_vector[1], left_b1b2_vector[0]) - np.arctan2(right_b1b2_vector[1], right_b1b2_vector[0])
        angle_degrees = np.degrees(angle)
        
        translation = np.array(transformed_left_points['B1']) - np.array(right_points['B1'])
        translation[0] += self.leg_distance  # 左右の脚の距離を考慮

        transformer_right = Transformation2D(origin=right_points['B1'], angle=angle_degrees, translation=translation)
        self.right_leg.apply_transformation(transformer_right)

        # 腰の位置と角度に基づいて全体を変換
        hip_transformer = Transformation2D(origin=(0, 0), angle=self.hip_angle, translation=self.hip_position)
        self.left_leg.apply_transformation(hip_transformer)
        self.right_leg.apply_transformation(hip_transformer)

    def compute_link_angles(self):
        # とりあえず、左の足を動かす
        self.left_leg.calculate_rotated_points()
        self.right_leg.calculate_rotated_points()

    def get_transformed_points(self):
        return {
            'left': self.left_leg.get_transformed_points(),
            'right': self.right_leg.get_transformed_points()
        }
    
    def get_rotated_points(self):
        return {
            'left': self.left_leg.get_rotated_points(),
            'right': self.right_leg.get_rotated_points()
        }

    @staticmethod
    def _angle_between_vectors(v1_start, v1_end, v2_start, v2_end):
        v1 = np.array(v1_end) - np.array(v1_start)
        v2 = np.array(v2_end) - np.array(v2_start)
        angle_rad = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
        angle_deg = np.degrees(angle_rad)
        return angle_deg if -180 <= angle_deg <= 180 else angle_deg - 360 * np.sign(angle_deg)
