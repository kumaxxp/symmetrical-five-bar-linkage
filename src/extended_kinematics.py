import numpy as np
import matplotlib.pyplot as plt
from linkage_kinematics import ForwardKinematics

def calculate_P3(P1, P2, L, theta):
    """
    P1からP2へのベクトルとP1からP3へのベクトルを用いてPの位置を計算する関数
    :param P1: P1の座標 (x1, y1)
    :param P2: P2の座標 (x2, y2)
    :param L: P1からP3へのベクトルの長さ
    :param theta: P1からP2へのベクトルとP1からP3へのベクトルとの間の角度 (度)
    :return: P3の座標 (x3, y3)
    """
    # ステップ1: ベクトルの定義
    x1, y1 = P1
    x2, y2 = P2
    v = np.array([x2 - x1, y2 - y1])
    
    # ベクトルvの長さを計算
    v_length = np.linalg.norm(v)
    
    # ステップ2: 内積と角度の関係
    # vの角度を計算
    phi_v = np.arctan2(v[1], v[0])  # Y軸からの角度
    theta_rad = np.radians(theta)    # thetaをラジアンに変換
    
    # phiを計算
    phi = phi_v + theta_rad
    
    # ステップ3: Pの位置の計算
    x3 = x1 + L * np.cos(phi)
    y3 = y1 + L * np.sin(phi)
    
    return (x3, y3)

class ExtendedKinematics(ForwardKinematics):
    def __init__(self, Yb, l, b, m, e, f):
        """
        初期化メソッド
        :param Yb: B1, B2 の Y 座標
        :param l: B1 から B2 までの距離
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        :param f: 追加リンクの長さ
        """
        super().__init__(Yb, l, b, m, e)
        self.f = f
        self.F = None  # 追加リンクの接続点

    def set_angles(self, theta1, theta2, thetaF):
        """
        モーターの角度設定
        :param theta1: B1-M1 の角度 (度)
        :param theta2: B2-M2 の角度 (度)
        :param thetaF: X-E-F の角度 (度)
        """
        super().set_angles(theta1, theta2)
        self.thetaF = thetaF

    def compute_forward_kinematics(self):
        """
        順運動学を計算し、Fの位置を含める
        """
        super().compute_forward_kinematics()
        self.F = self.calculate_F()

    def calculate_F(self):
        """
        EからFへの位置を計算
        """
        if self.E is None or self.X is None:
            raise ValueError("EまたはX位置が計算されていません。")

        return calculate_P3(self.E, self.X, self.f, self.thetaF)

    def calculate(self):
        """
        順運動学の計算を行い、結果を返す
        :return: 順運動学の結果
        """
        result = super().calculate()
        result["F"] = self.F  # Fの位置を結果に追加
        return result

def plot_extended_kinematics(ek):
    """
    拡張運動学の結果をプロットする
    :param ek: ExtendedKinematics インスタンス
    """
    result = ek.calculate()

    if isinstance(result, str):
        print(result)
        return

    B1, M1, X, M2, B2, E, X1, X2, F = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["X1"], result["X2"], result["F"]

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
    if E is not None and F is not None:
        plt.plot([E[0], F[0]], [E[1], F[1]], 'c-o', label='E-F')

    # 各点をプロット
    for point, color, label in zip([B1, M1, X, M2, B2, E, F],
                                    ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan'],
                                    ['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F']):
        if point is not None:
            plt.scatter(*point, color=color, zorder=5)
            plt.text(point[0], point[1], label, fontsize=12, ha='right')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Extended Kinematics Visualization')
    plt.grid(True)
    plt.legend()
    plt.xlim(-600, 600)
    plt.ylim(-1000, 200)
    plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200    # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200    # X-E の距離
    f = 150  # 追加リンクの長さ

    # 拡張運動学インスタンス作成
    ek = ExtendedKinematics(Yb, l, b, m, e, f)

    # モーターの角度を設定 (サンプル値)
    theta1 = -45 + 30
    theta2 = -135 + 30
    thetaF = -60  # X-Fの角度
    ek.set_angles(theta1, theta2, thetaF)
    ek.compute_forward_kinematics()

    # 拡張運動学の結果をプロット
    plot_extended_kinematics(ek)
