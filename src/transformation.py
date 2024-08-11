import numpy as np

class Transformation2D:
    def __init__(self, origin=(0, 0), angle=0):
        """
        初期化メソッド
        :param origin: 回転の中心点の座標 (x, y)。この点が新しい (0, 0) になります。
        :param angle: 回転角度（度）
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
        指定された点を回転させ、新しい原点に移動させるメソッド
        :param point: 変換する点 (x, y)
        :return: 変換後の点 (x', y')
        """
        # 原点を (0, 0) に移動
        translated_point = np.array(point) - self.origin
        
        # 回転
        rotated_point = np.dot(self.rotation_matrix, translated_point)
        
        return rotated_point

    def inverse_transform(self, point):
        """
        変換の逆変換を行うメソッド
        :param point: 逆変換する点 (x, y)
        :return: 逆変換後の点 (x', y')
        """
        # 逆回転
        inverse_rotation_matrix = self.rotation_matrix.T  # 転置行列が逆回転行列
        rotated_back_point = np.dot(inverse_rotation_matrix, np.array(point))
        
        # 元の座標系に戻す
        return rotated_back_point + self.origin
