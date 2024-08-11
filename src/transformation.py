import numpy as np

class Transformation2D:
    def __init__(self, origin=(0, 0), angle=0):
        self.origin = np.array(origin)
        self.angle = np.radians(angle)
        self.rotation_matrix = self.calculate_rotation_matrix()

    def calculate_rotation_matrix(self):
        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        return np.array([[cos_angle, -sin_angle],
                         [sin_angle, cos_angle]])

    def transform(self, point):
        # 新しい原点に移動
        translated_point = np.array(point) - self.origin
        # 回転の中心を原点に移動
        centered_point = translated_point - self.origin
        
        # 回転
        rotated_point = np.dot(self.rotation_matrix, centered_point)
        
        # 回転の中心を元に戻す
        return rotated_point + self.origin

    def inverse_transform(self, point):
        # 回転の中心を原点に移動
        centered_point = np.array(point) - self.origin
        
        # 逆回転
        inverse_rotation_matrix = self.rotation_matrix.T
        rotated_back_point = np.dot(inverse_rotation_matrix, centered_point)
        
        # 回転の中心を元に戻す
        translated_back_point = rotated_back_point + self.origin
        
        # 新しい原点から元の座標系に戻す
        return translated_back_point + self.origin
