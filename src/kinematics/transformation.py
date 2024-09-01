import numpy as np

class Transformation2D:
    def __init__(self, origin=(0, 0), angle=0, translation=(0, 0)):
        self.origin = np.array(origin)
        self.angle = np.radians(angle)
        self.translation = np.array(translation)

    def transformPoint(self, point):
        point = np.array(point)
        # 原点を中心に回転
        rotated = self.rotate(point - self.origin)
        # 平行移動を適用
        translated = rotated + self.translation + self.origin
        return translated

    def rotate(self, point):
        cos_theta = np.cos(self.angle)
        sin_theta = np.sin(self.angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta],
                                    [sin_theta, cos_theta]])
        return np.dot(rotation_matrix, point)
