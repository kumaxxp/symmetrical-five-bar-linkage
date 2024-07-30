import numpy as np
import matplotlib.pyplot as plt
from linkage_inverse_kinematics import InverseKinematics

class KinematicsAnalyzer:
    def __init__(self, Yb, l, b, m, e):
        # InverseKinematicsクラスのインスタンスを作成
        self.ik = InverseKinematics(Yb, l, b, m, e)
    
    def analyze_reachability(self, x_range, y_range, step_size):
        results = []
        
        # エンドエフェクタの位置を変化させながら解析
        for x in np.arange(x_range[0], x_range[1] + step_size, step_size):
            for y in np.arange(y_range[0], y_range[1] + step_size, step_size):
                E = np.array([x, y])
                self.ik.set_endeffector(E)
                self.ik.compute_inverse_kinematics()
                
                if self.ik.valid_combinations:
                    for combination in self.ik.valid_combinations:
                        M1, X, M2 = combination
                        theta1, theta2 = self.calculate_angles(M1, X, M2)
                        results.append([x, y, theta1, theta2, True])
                else:
                    results.append([x, y, None, None, False])
        
        return np.array(results)
    
    def calculate_angles(self, M1, X, M2):
        # 角度の計算
        # B1-M1とX-M1の角度
        theta1 = np.arctan2(M1[1] - self.ik.B1[1], M1[0] - self.ik.B1[0])
        # B2-M2とX-M2の角度
        theta2 = np.arctan2(M2[1] - self.ik.B2[1], M2[0] - self.ik.B2[0])
        return theta1, theta2
    
    def plot_reachability(self, results):
        results = np.array(results)
        
        # 到達可能な点をプロット
        reachable_points = results[results[:, 4] == True]
        unreachable_points = results[results[:, 4] == False]
        
        plt.figure(figsize=(10, 6))
        plt.scatter(reachable_points[:, 0], reachable_points[:, 1], c='blue', label='Reachable', marker='o')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Reachability Analysis')
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200    # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200    # X-E の距離

    # 解析クラスのインスタンス作成
    analyzer = KinematicsAnalyzer(Yb, l, b, m, e)
    
    # 解析範囲とステップサイズを設定
    x_range = (-500, 500)
    y_range = (-1000, 0)
    step_size = 50

    # 解析を実行
    results = analyzer.analyze_reachability(x_range, y_range, step_size)
    
    # 結果をプロット
    analyzer.plot_reachability(results)
