import numpy as np
from kinematics.extended_kinematics import ExtendedKinematics

class Hip:
    def __init__(self, ground_y=0):
        self.left_leg = None
        self.right_leg = None
        self.ground_y = ground_y

    def set_leg_param(self, b, m, e, f, B1, B2):
        """
        両足のパラメータを設定する
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        :param f: 追加リンクの長さ
        :param B1: B1の座標 (x, y)
        :param B2: B2の座標 (x, y)
        """

        self.left_leg = ExtendedKinematics(b, m, e, f, B1, B2)
        self.right_leg = ExtendedKinematics(b, m, e, f, B1, B2)

    def set_leg_angles(self, leg, theta1, theta2, thetaF):
        if leg not in ['left', 'right']:
            raise ValueError("Invalid leg name. Must be 'left' or 'right'.")
        getattr(self, f"{leg}_leg").set_angles(theta1, theta2, thetaF)

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

    def align_legs_to_ground(self):
        left_points = self.left_leg.get_original_points()
        right_points = self.right_leg.get_original_points()

        # 左脚を地面に合わせて回転
        self.left_leg.calculate_rotated_points(left_points['E'], left_points['F'], (0, 0), (1, 0))

        # 回転後の左脚のB1, B2の位置を取得
        rotated_left_points = self.left_leg.get_rotated_points()

        # 右脚をB1, B2に合わせて回転と平行移動
        self.right_leg.calculate_rotated_points(left_points['B2'], left_points['B1'], rotated_left_points['B2'], rotated_left_points['B1'])
        
        self.right_leg.calculate_translate_points(rotated_left_points['B2'])

    @staticmethod
    def _angle_between_vectors(v1_start, v1_end, v2_start, v2_end):
        v1 = np.array(v1_end) - np.array(v1_start)
        v2 = np.array(v2_end) - np.array(v2_start)
        angle_rad = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
        angle_deg = np.degrees(angle_rad)
        return angle_deg if -180 <= angle_deg <= 180 else angle_deg - 360 * np.sign(angle_deg)
    
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