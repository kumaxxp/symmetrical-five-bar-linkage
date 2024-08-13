import numpy as np

class Transformation2D:
    def __init__(self, origin, angle, translation=(0, 0)):
        self.origin = np.array(origin)
        self.angle = angle
        self.translation = np.array(translation)

    def transform(self, point):
        point = np.array(point) - self.origin
        rotated = self._rotate(point)
        return rotated + self.origin + self.translation

    def _rotate(self, point):
        cos_theta = np.cos(np.radians(self.angle))
        sin_theta = np.sin(np.radians(self.angle))
        return np.array([
            point[0] * cos_theta - point[1] * sin_theta,
            point[0] * sin_theta + point[1] * cos_theta
        ])
