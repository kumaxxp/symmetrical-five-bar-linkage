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
        theta1 = np.arctan2(M1[1] - self.ik.B1[1], M1[0] - self.ik.B1[0])
        theta2 = np.arctan2(M2[1] - self.ik.B2[1], M2[0] - self.ik.B2[0])
        return theta1, theta2
    
    def plot_reachability(self, results, default_position):
        results = np.array(results)
        
        # 到達可能な点をプロット
        reachable_points = results[results[:, 4] == True]
        unreachable_points = results[results[:, 4] == False]
        
        plt.figure(figsize=(12, 8))
        
        # 到達可能な点をプロット
        plt.scatter(reachable_points[:, 0], reachable_points[:, 1], c='blue', label='Reachable', marker='o')
        
        # デフォルトの位置で逆運動学のリンクをプロット
        self.ik.set_endeffector(default_position)
        self.ik.compute_inverse_kinematics()
        if self.ik.valid_combinations:
            for combination in self.ik.valid_combinations:
                M1, X, M2 = combination
                plt.plot([self.ik.B1[0], M1[0]], [self.ik.B1[1], M1[1]], 'ro-')
                plt.plot([M1[0], X[0]], [M1[1], X[1]], 'go-')
                plt.plot([X[0], M2[0]], [X[1], M2[1]], 'bo-')
                plt.plot([M2[0], self.ik.B2[0]], [M2[1], self.ik.B2[1]], 'mo-')
                plt.plot([self.ik.B1[0], self.ik.B2[0]], [self.ik.B1[1], self.ik.B2[1]], 'k--')
                plt.plot([X[0], self.ik.E[0]], [X[1], self.ik.E[1]], 'ko-')
                
                plt.plot(self.ik.B1[0], self.ik.B1[1], 'ro')  # B1
                plt.plot(self.ik.B2[0], self.ik.B2[1], 'ro')  # B2
                plt.plot(M1[0], M1[1], 'go')  # M1
                plt.plot(X[0], X[1], 'bo')  # X
                plt.plot(M2[0], M2[1], 'mo')  # M2
                plt.plot(self.ik.E[0], self.ik.E[1], 'ko')  # E

        # グリッド幅に合わせた矢印の長さ
        grid_width = 50  # グリッドの幅
        arrow_length = min(grid_width / 2, 50)  # 短くするために調整
        
        for i in range(len(results)):
            if results[i, 4] == True and not np.isnan(results[i, 2]) and not np.isnan(results[i, 3]):
                # θ1に基づく矢印
                plt.arrow(results[i, 0], results[i, 1], 
                          arrow_length * np.cos(results[i, 2]), arrow_length * np.sin(results[i, 2]),
                          head_width=10, head_length=10, fc='blue', ec='blue')
                # θ2に基づく矢印
                plt.arrow(results[i, 0], results[i, 1], 
                          arrow_length * np.cos(results[i, 3]), arrow_length * np.sin(results[i, 3]),
                          head_width=10, head_length=10, fc='green', ec='green')
        
        # 軸とラベルの設定
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Reachability Analysis with Default Position Overlaid')
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
    x_range = (-700, 700)
    y_range = (-1500, 1500)
    step_size = 50

#    theta1_range = (-180, 0)  # 角度範囲
#    theta2_range = (-180, 0)  # 角度範囲
#    step_size = 4            # ステップサイズ

    # 点Eを順運動でプロット    
#    E_points = analyzer.analyze_angles(theta1_range, theta2_range, step_size)
#    analyzer.plot_results(E_points)

    # 点Eを逆運動でプロット
    # 解析を実行
    results = analyzer.analyze_reachability(x_range, y_range, step_size)
    
    # デフォルト位置
    default_position = np.array([-20, -660])
    
    # 結果をプロット
    analyzer.plot_reachability(results, default_position)
