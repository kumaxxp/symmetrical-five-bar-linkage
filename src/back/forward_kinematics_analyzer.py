import numpy as np
import matplotlib.pyplot as plt
from linkage_kinematics import ForwardKinematics

class ForwardKinematicsAnalyzer:
    def __init__(self, Yb, l, b, m, e):
        self.fk = ForwardKinematics(Yb, l, b, m, e)
    
    def analyze_angles(self, theta1_range, theta2_range, step_size):
        E_points = []
        
        for theta1 in np.arange(theta1_range[0], theta1_range[1] + step_size, step_size):
            for theta2 in np.arange(theta2_range[0], theta2_range[1] + step_size, step_size):
                self.fk.set_angles(theta1, theta2)
                self.fk.compute_forward_kinematics()
                result = self.fk.calculate()

                E = result.get("E")
                if E is not None:
                    E_points.append((E[0], E[1], theta1, theta2))
        
        return np.array(E_points)
    
    def plot_results(self, E_points):
        plt.figure(figsize=(10, 6))
        plt.scatter(E_points[:, 0], E_points[:, 1], c='blue', marker='o', label='E Points')
        
#        for point in E_points:
#            plt.annotate(f'θ1: {point[2]:.1f}, θ2: {point[3]:.1f}', (point[0], point[1]))
        
        plt.xlabel('X coordinate of E')
        plt.ylabel('Y coordinate of E')
        plt.title('Forward Kinematics Analysis')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200     # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200     # X-E の距離
    
    analyzer = ForwardKinematicsAnalyzer(Yb, l, b, m, e)
    
    theta1_range = (-180, 0)  # 角度範囲
    theta2_range = (-180, 0)  # 角度範囲
    step_size = 4            # ステップサイズ
    
    E_points = analyzer.analyze_angles(theta1_range, theta2_range, step_size)
    analyzer.plot_results(E_points)
