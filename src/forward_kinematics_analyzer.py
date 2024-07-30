import numpy as np
import matplotlib.pyplot as plt
from linkage_kinematics import ForwardKinematics

class ForwardKinematicsAnalyzer:
    def __init__(self, Yb, l, b, m, e):
        # ForwardKinematicsクラスのインスタンスを作成
        self.fk = ForwardKinematics(Yb, l, b, m, e)
    
    def analyze_angles(self, theta1_range, theta2_range, step_size):
        theta1_values = np.arange(theta1_range[0], theta1_range[1] + step_size, step_size)
        theta2_values = np.arange(theta2_range[0], theta2_range[1] + step_size, step_size)

        E_points = []

        for theta1 in theta1_values:
            for theta2 in theta2_values:
                # 順運動学を使って点 E を計算
                self.fk.set_angles(theta1, theta2)
                E = self.fk.calculate_E()
                
                if E is not None:  # 計算が成功した場合のみプロット
                    E_points.append((theta1, theta2, E[0], E[1]))
        
        return np.array(E_points)

    def plot_results(self, E_points):
        plt.figure(figsize=(12, 6))
        
        # 点Eのプロット
        plt.scatter(E_points[:, 2], E_points[:, 3], c='blue', label='Calculated E')
        
        # 軸ラベルとタイトル
        plt.xlabel('X coordinate of E')
        plt.ylabel('Y coordinate of E')
        plt.title('Points E for varying angles θ1 and θ2')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200    # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200    # X-E の距離
    
    # ForwardKinematicsAnalyzerクラスのインスタンスを作成
    analyzer = ForwardKinematicsAnalyzer(Yb, l, b, m, e)
    
    # 角度範囲とステップサイズを設定
    theta1_range = (0, 360)  # 角度範囲（度）
    theta2_range = (0, 360)  # 角度範囲（度）
    step_size = 1  # ステップサイズ（度）
    
    # 解析を実施
    E_points = analyzer.analyze_angles(theta1_range, theta2_range, step_size)
    
    # 結果をプロット
    analyzer.plot_results(E_points)
