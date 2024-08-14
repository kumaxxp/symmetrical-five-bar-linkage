import numpy as np
import matplotlib.pyplot as plt
from linkage_kinematics import ForwardKinematics

def calculate_P3(P1, P2, L, theta):
    """
    P1からP2へのベクトルとP1からP3へのベクトルを用いてP3の位置を計算する関数
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
        self.F = None
        self.thetaF = 0
        self.transformer = None
        self.transformed_points = {}

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
        result["F"] = self.F
        return result

    def format_result(self):
        """
        計算結果をフォーマットして返す
        :return: フされた結果
        """
        return {
            "B1": self.B1,
            "M1": self.M1,
            "X": self.X,
            "M2": self.M2,
            "B2": self.B2,
            "E": self.E,
            "F": self.F,
            "X1": self.X1,
            "X2": self.X2
        }

    def apply_transformation(self, transformer):
        """
        座標変換を適用する
        :param transformer: 変換器オブジェクト
        """
        self.transformer = transformer
        self.transformed_points = {
            key: self.transformer.transform(value)
            for key, value in self.format_result().items()
        }

    def get_transformed_points(self):
        """
        変換された座標を取得する
        :return: 変換された座標
        """
        return self.transformed_points

    def get_original_points(self):
        """
        元の座標を取得する
        :return: 元の座標
        """
        return self.format_result()

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

    # 結果を表示
    result = ek.calculate()
    for key, value in result.items():
        print(f"{key}: {value}")
