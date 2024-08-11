import numpy as np

class Transformation2D:
    def __init__(self, origin=(0, 0), angle=0):
        """
        初期化メソッド
        :param origin: 原点の座標 (x, y)
        :param angle: 回転角度度)
        """
        self.origin = np.array(origin)
        self.angle = np.radians(angle)  # ラジアンに変換
        self.rotation_matrix = self.calculate_rotation_matrix()

    def calculate_rotation_matrix(self):
        """回転行列を計算するメソッド"""
        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        return np.array([[cos_angle, -sin_angle],
                         [sin_angle, cos_angle]])

    def transform(self, point):
        """
        指定された点を回転させ、原点を移動させるメソッド
        :param point: 変換点 (x, y)
        :return: 変換後の点 (x', y')
        """
        # 原点を中心に回転
        translated_point = np.array(point) - self.origin
        rotated_point = np.dot(self.rotation_matrix, translated_point)
        return rotated_point + self.origin
