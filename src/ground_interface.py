import math
from extended_kinematics import ExtendedKinematics
from transformation import Transformation2D

class Ground:
    def __init__(self, slope_angle=0):
        self._slope_angle = slope_angle

    def get_slope_angle(self):
        return self._slope_angle

    def set_slope_angle(self, angle):
        self._slope_angle = angle

class LegGroundInterface:
    def __init__(self, leg: ExtendedKinematics, ground: Ground, contact_link_index: int):
        self.leg = leg
        self.ground = ground
        self.contact_link_index = contact_link_index

    def align_leg_to_ground(self):
        # 地面の傾斜角度を取得
        ground_angle = self.ground.get_slope_angle()
        
        # 接地リンクの角度を取得
        link_angle = self.leg.get_link_angle(self.contact_link_index)
        
        # 接地リンクと地面の角度差を計算
        angle_difference = ground_angle - link_angle
        
        # 脚全体を回転させるための変換を作成
        transformation = Transformation2D(
            origin=(0, 0),  # 原点を中心に回転
            angle=angle_difference,
            translation=(0, 0)  # 平行移動なし
        )
        
        # 変換を適用
        self.leg.apply_transformation(transformation)

    def get_alignment_transformation(self):
        ground_angle = self.ground.get_slope_angle_angle = self.leg.get_link_angle(self.contact_link_index)
        angle_difference = ground_angle - link_angle
        
        return Transformation2D(
            origin=(0, 0),
            angle=angle_difference,
            translation=(0, 0)
        )
