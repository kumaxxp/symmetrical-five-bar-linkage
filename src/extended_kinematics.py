import numpy as np
import matplotlib.pyplot as plt
from linkage_kinematics import ForwardKinematics

class ExtendedKinematics(ForwardKinematics):
    def __init__(self, Yb, l, b, m, e, additional_link_length):
        """
        初期化メソッド
        :param Yb: B1, B2 の Y 座標
        :param l: B から B2 までの距離
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        :param additional_link_length: 追加リンクの長さ
        """
        super().__init__(Yb, l, b, m, e)
        self.additional_link_length = additional_link_length
        self.M3 = None  # 追加リンクの接続点

    def set_angles(self, theta1, theta2, theta3):
        """
        モーターの角度設定
        :param theta1: B1-M1 の角度 (度)
        :param theta2: B2-M2 の角度 (度)
        :param theta3: M2-M3 の角度 (度)
        """
        super().set_angles(theta1, theta2)
        self.theta3 = theta3

    def compute_forward_kinematics(self):
        """
        順運動学を計算し、M3の位置を含める
        """
        super().compute_forward_kinematics()
        self.calculate_M3()

    def calculate_M3(self):
        """
        M2からM3への位置を計算        """
        theta3_rad = np.radians(self.theta3)
        self.M3 = self.M2 + np.array([self.additional_link_length * np.cos(theta3_rad),
                                       self.additional_link_length * np.sin(theta3_rad)])

    def calculate(self):
        """
        順運動学の計算を行い、結果を返す
        :return: 順運動学の結果
        """
        result = super().calculate()
        result["M3"] = self.M3  # M3の位置を結果に追加
        return result

def plot_extended_kinematics(ek):
    """
    拡張運動学の結果をプロする
    :param ek: ExtendedKinematics インスタンス
    """
    result = ek.calculate()

    if isinstance(result, str):
        print(result)
        return

    B1, M1, X, M2, B2, E, X1, X2, M3 = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["X1"], result["X2"], result["M3"]

    plt.figure()
    if B1 is not None and M1 is not None:
        plt.plot([B1[0], M1[0]], [B1[1], M1[1]], 'r-o', label='B1-M1')
    if X is not None and M1 is not None:
        plt.plot([M1[0], X[0]], [M1[1], X[1]], 'b-o', label='M1-X')
    if X is not None and M2 is not None:
        plt.plot([X[0], M2[0]], [X[1], M2[1]], 'g-o', label='X-M2')
    if M2 is not None and B2 is not None:
        plt.plot([M2[0], B2[0]], [M2[1], B2[1]], 'y-o', label='M2-B2')
    if X is not None and E is not None:
        plt.plot([X[0], E[0]], [X[1], E[1]], 'mo-', label='X-E')
    if M2 is not None and M3 is not None:
        plt.plot([M2[0], M3[0]], [M2[1], M3[1]], 'c-o', label='M2-M3')

    # 各点をプロット
    for point, color, label in zip([B1, M1, X, M2, B2, E, M3],
                                    ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan'],
                                    ['B1', 'M1', 'X', 'M2', 'B2', 'E', 'M3']):
        if point is not None:
            plt.scatter(*point, color=color, zorder=5)
            plt.text(point[0], point[1], label, fontsize=12, ha='right')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Extended Kinematics Visualization')
    plt.grid(True)
    plt.legend()
    plt.xlim(-600, 600)
    plt.ylim(-1000,200)
    plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200    # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200    # X-E の距離
    additional_link_length = 150  # 追加リンクの長さ

    # 拡張運動学インスタンス作成
    ek = ExtendedKinematics(Yb, l, b, m, e, additional_link_length)

    # モーターの角度を設定 (サンプル値)
    theta1 = -45
    theta2 = -135
    theta3 = 30  # 追加リンクの角度
    ek.set_angles(theta1, theta2, theta3)
    ek.compute_forward_kinematics()

    # 拡張運動学の結果をプロット
    plot_extended_kinematics(ek)
