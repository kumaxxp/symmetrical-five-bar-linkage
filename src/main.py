import numpy as np
import matplotlib.pyplot as plt
from forward_kinematics_analyzer import ForwardKinematicsAnalyzer
from kinematics_analyzer import KinematicsAnalyzer

if __name__ == "__main__":
    # 初期値設定
    Yb = 100
    l = 200
    b = 200
    m = 400
    e = 200
    theta1_range = (0, 360)  # θ1の範囲
    theta2_range = (0, 360)  # θ2の範囲
    step_size = 10  # ステップサイズ

    # 逆運動学の解析
    x_range = (-400, 400)
    y_range = (-800, 0)

    ik_analyzer = KinematicsAnalyzer(Yb, l, b, m, e)
    ik_results = ik_analyzer.analyze_reachability(x_range, y_range, step_size)

    # 順運動学の解析
    fk_analyzer = ForwardKinematicsAnalyzer(Yb, l, b, m, e)
    fk_results = fk_analyzer.analyze_angles(theta1_range, theta2_range, step_size)

    # プロット
    plt.figure(figsize=(12, 6))
    
    # 逆運動学の結果をプロット
    plt.scatter(ik_results[:, 0], ik_results[:, 1], c='blue', s=10, label='Inverse Kinematics Reachable Points')
    
    # 順運動学の結果をプロット
    plt.scatter(fk_results[:, 0], fk_results[:, 1], c='red', s=10, label='Forward Kinematics Reachable Points')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Reachable Points from Inverse and Forward Kinematics')
    plt.legend()
    plt.grid(True)
    plt.show()
