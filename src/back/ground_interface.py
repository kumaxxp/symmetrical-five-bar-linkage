import math
import numpy as np
from extended_kinematics import ExtendedKinematics
from transformation import Transformation2D

class Ground:
    def __init__(self, slope_angle=0, friction_coefficient=0.5):
        self._slope_angle = slope_angle
        self._friction_coefficient = friction_coefficient

    def get_slope_angle(self):
        return self._slope_angle

    def set_slope_angle(self, angle):
        self._slope_angle = angle

    def get_friction_coefficient(self):
        return self._friction_coefficient

    def set_friction_coefficient(self, coefficient):
        self._friction_coefficient = coefficient

    def get_height(self, x):
        return x * math.tan(self._slope_angle)

class LegGroundInterface:
    def __init__(self, leg: ExtendedKinematics, ground: Ground, contact_link_index: int):
        self.leg = leg
        self.ground = ground
        self.contact_link_index = contact_link_index
        self.contact_point = None

    def align_leg_to_ground(self):
        ground_angle = self.ground.get_slope_angle()
        link_angle = self.leg.get_link_angle(self.contact_link_index)
        angle_difference = ground_angle - link_angle
        
        transformation = Transformation2D(
            origin=(0, 0),
            angle=angle_difference,
            translation=(0, 0)
        )
        
        self.leg.apply_transformation(transformation)

    def get_alignment_transformation(self):
        ground_angle = self.ground.get_slope_angle()
        link_angle = self.leg.get_link_angle(self.contact_link_index)
        angle_difference = ground_angle - link_angle
        
        return Transformation2D(
            origin=(0, 0),
            angle=angle_difference,
            translation=(0, 0)
        )

    def calculate_reaction_force(self):
        if self.contact_point is None:
            return np.array([0, 0])

        # 簡単な反力モデル（重力に基づく）
        gravity = 9.81  # m/s^2
        mass = 1  # kg (仮の質量)
        normal_force = mass * gravity * math.cos(self.ground.get_slope_angle())
        friction_force = normal_force * self.ground.get_friction_coefficient()

        # 地面に平行な方向と垂直な方向の力を計算
        parallel_force = friction_force * math.cos(self.ground.get_slope_angle())
        perpendicular_force = normal_force

        return np.array([parallel_force, perpendicular_force])

    def detect_contact(self):
        # 接触検出のロジックをここに実装
        # 例: 最も低い点が地面より下にあれば接触とみなす
        points = self.leg.get_transformed_points()
        lowest_point = min(points.values(), key=lambda p: p[1])
        if lowest_point[1] <= 0:  #平面のy座標を0と仮定
            self.contact_point = lowest_point
            return True
        else:
            self.contact_point = None
            return False

if __name__ == "__main__":
    # 使用例
    ground = Ground(slope_angle=math.radians(5), friction_coefficient=0.6)
    leg = ExtendedKinematics(200, 400, 200, 150, (100, -100), (-100, -100))
    interface = LegGroundInterface(leg, ground, 0)

    # 初期姿勢を設定
    leg.set_angles(-45, -135, -60)
    leg.compute_forward_kinematics()

    # 地面との接触を検出し、脚を地わせて調整
    interface.align_leg_to_ground()

    # 反力を計算し、脚に適用
    reaction_force = interface.calculate_reaction_force()
    #leg.apply_external_force(reaction_force)

    # 外力を考慮して運動学を再計算
    leg.compute_forward_kinematics()

    # 結果を表示
    result = leg.get_transformed_points()
    for key, value in result.items():
        print(f"{key}: {value}")
