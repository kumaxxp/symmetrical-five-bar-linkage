import numpy as np
from kinematics.extended_kinematics import ExtendedKinematics

class Hip:
    def __init__(self, ground_y=0):
        self.left_leg = None
        self.right_leg = None
        self.ground_y = ground_y
        self.weights = {}  # 各点の重量を保存する辞書

        self.center_com = None
        self.left_com = None
        self.right_com = None

    def set_leg_param(self, b, m, e, f, B1, B2, W1, W2, w):
        """
        両足のパラメータを設定する
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        :param f: 追加リンクの長さ
        :param B1: B1の座標 (x, y)
        :param B2: B2の座標 (x, y)
        :param W1: W1の座標 (x, y)
        :param W2: W2の座標 (x, y)
        :param w: リンクとワイヤーの距離
        """

        self.left_leg = ExtendedKinematics(b, m, e, f, B1, B2, W1, W2, w)
        self.right_leg = ExtendedKinematics(b, m, e, f, B1, B2, W1, W2, w)

    def set_leg_angles(self, leg, theta1, theta2, thetaF):
        if leg not in ['left', 'right']:
            raise ValueError("Invalid leg name. Must be 'left' or 'right'.")
        getattr(self, f"{leg}_leg").setAngles(theta1, theta2, thetaF)

    def compute_forward_kinematics(self):
        self.left_leg.computeForwardKinematics()
        self.right_leg.computeForwardKinematics()

    def get_rotated_points(self):
        return {
            'left': self.left_leg.getRotatedPoints(),
            'right': self.right_leg.getRotatedPoints()
        }

    def computeLinkAngles(self):
        self.left_leg.computeLinkAngles()
        self.right_leg.computeLinkAngles()

    def set_weights(self, weights):
        """
        各点の重量を設定する
        :param weights: 点の名前と重量のペアを含む辞書
        例: {'B1': 1, 'B2': 1, 'E': 1}
        """
        self.weights = weights

    def calculate_leg_center_of_mass(self, leg):
        """
        指定された脚の重心を計算する
        """
        points = getattr(self, f"{leg}_leg").getRotatedPoints()
        weight = 0
        total_weight = 0
        weighted_sum = np.array([0.0, 0.0])

        for point_name, coordinates in points.items():
            if point_name in self.weights:
                weight = self.weights[point_name]
                total_weight += weight
                weighted_sum += np.array(coordinates) * weight

        if total_weight == 0:
            return None
        return weighted_sum / total_weight

    def calculate_total_center_of_mass(self):
        """
        全体の重心を計算する
        """
        self.left_com = self.calculate_leg_center_of_mass('left')
        self.right_com = self.calculate_leg_center_of_mass('right')

        if self.left_com is None or self.right_com is None:
            return None

        total_weight = sum(self.weights.values())
        weighted_sum = self.left_com * sum(self.weights.values()) / 2 + self.right_com * sum(self.weights.values()) / 2

        self.center_com = weighted_sum / total_weight

    def align_legs_to_ground(self, rotation_option):
        left_points = self.left_leg.getOriginalPoints()
        right_points = self.right_leg.getOriginalPoints()

        if rotation_option == "ground":
            # 左脚を地面に合わせて回転
            self.left_leg.calculateRotatedPoints(left_points['E'], left_points['F'], (0, 0), (1, 0))
        elif rotation_option == "toe":
            # つま先側を地面に合わせて回転
            self.left_leg.calculateRotatedPoints(left_points['F'], left_points['I'], (0, 0), (1, 0))
        elif rotation_option == "heel":
            # かかと側を地面に合わせて回転
            self.left_leg.calculateRotatedPoints(left_points['H'], left_points['E'], (0, 0), (1, 0))

        # 回転後の左脚のB1, B2の位置を取得
        rotated_left_points = self.left_leg.getRotatedPoints()

        # 右脚をB1, B2に合わせて回転と平行移動
        self.right_leg.calculateRotatedPoints(left_points['B2'], left_points['B1'], rotated_left_points['B2'], rotated_left_points['B1'])
        
        self.right_leg.calculateTranslatePoints(rotated_left_points['B2'])

    @staticmethod
    def _angle_between_vectors(v1_start, v1_end, v2_start, v2_end):
        v1 = np.array(v1_end) - np.array(v1_start)
        v2 = np.array(v2_end) - np.array(v2_start)
        angle_rad = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
        angle_deg = np.degrees(angle_rad)
        return angle_deg if -180 <= angle_deg <= 180 else angle_deg - 360 * np.sign(angle_deg)
    
        # 重心の計算と表示
        left_com = self.calculate_leg_center_of_mass('left')
        right_com = self.calculate_leg_center_of_mass('right')
        total_com = self.calculate_total_center_of_mass()

        print(f"Left leg center of mass: {left_com}")
        print(f"Right leg center of mass: {right_com}")
        print(f"Total center of mass: {total_com}")

    def display_length_info(self):
        left_info = self.left_leg.getLengthInfo()
        right_info = self.right_leg.getLengthInfo()
        left_spring_info = self.left_leg.getLengthInfo('spring')
        right_spring_info = self.right_leg.getLengthInfo('spring')
        left_diff_info = self.left_leg.getLengthInfo('diff')
        right_diff_info = self.right_leg.getLengthInfo('diff')
        return {
            'left': left_info,
            'right': right_info,
            'left spring': left_spring_info,
            'right spring': right_spring_info,
            'left diff': left_diff_info,
            'right diff': right_diff_info
        }


if __name__ == "__main__":
    # extended_kinematicsと同様にデフォルトのリンクの数値のメカ構造を生成
    # Y=0の地面に対して、左脚の足先E-Fが平行になるように回転させてリンクを変化させる

    hip = Hip(ground_y=0)

    B1 = (50, -100 + 200)
    B2 = (-50, -100 + 200)
    W1 = (60, -100 + 200)
    W2 = (-60, -100 + 200)
    hip.set_leg_param(150, 200, 150, 150, B1, B2, W1, W2, 20)

    #の設定
    weights = {
        'B1': 1, 'B2': 1,
        'M1': 2, 'M2': 2,
        'X': 3,
        'E': 1, 'F': 1
    }
    hip.set_weights(weights)

    initial_angles = {
        'left': {'theta1': -50, 'theta2': -120, 'thetaF': -60},
        'right': {'theta1': -50, 'theta2': -120, 'thetaF': -60}
    }

    for leg in ['left', 'right']:
        hip.set_leg_angles(leg, **initial_angles[leg])

    hip.compute_forward_kinematics()
    hip.align_legs_to_ground()
