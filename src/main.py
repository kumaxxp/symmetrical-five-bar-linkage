import numpy as np
from linkage_inverse_kinematics import InverseKinematics

# サンプルの値を設定
Yb = 100  # B1, B2 の Y 座標
l = 200    # B1 から B2 までの距離
b = 200    # B1-M1 および B2-M2 のリンク長
m = 400    # M1-X および M2-X のリンク長
e = 200    # X-E の距離

# InverseKinematicsクラスのインスタンスを作成
ik = InverseKinematics(Yb, l, b, m, e)

# テストするエンドエフェクタの位置
test_positions = [
    np.array([0, 0]),
    np.array([100, -200]),
    np.array([-300, -600]),
]

for pos in test_positions:
    ik.set_endeffector(pos)
    valid_combinations = ik.compute_inverse_kinematics()
    print(f"Testing position: {pos}")
    print("Valid combinations:", valid_combinations)
