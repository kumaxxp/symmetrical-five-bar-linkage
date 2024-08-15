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
    def __init__(self, b, m, e, f, B1, B2):
        """
        初期化メソッド
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        :param f: 追加リンクの長さ
        :param B1: B1の座標 (x, y)
        :param B2: B2の座標 (x, y)
        """
        super().__init__(b, m, e, B1, B2)
        self.f = f
        self.F = None
        self.thetaF = 0
        self.points = {}
        self.transformed_points = {}
        self.B1 = B1
        self.B2 = B2
        self.theta1 = 0
        self.theta2 = 0

    def set_angles(self, theta1, theta2, thetaF):
        """
        モーターの角度設定
        :param theta1: B1-M1 の角度 (度)
        :param theta2: B2-M2 の角度 (度)
        :param thetaF: X-E-F の角度 (度)
        """
        super().set_angles(theta1, theta2)
        self.thetaF = thetaF
        self.theta1 = theta1
        self.theta2 = theta2

    def compute_forward_kinematics(self):
        """
        順運動学を計算し、Fの位置を含める
        """
        super().compute_forward_kinematics()
        self.F = self.calculate_F()
        self.points = self.format_result()
        self.transformed_points = self.points.copy()

    def calculate_F(self):
        """
        EからFへの位置を計算
        """
        if self.E is None or self.X is None:
            raise ValueError("EまたはX位置が計算されていません。")

        return calculate_P3(self.E, self.X, self.f, self.thetaF)

    def calculate(self):
        """
        順運動学の計算を行い、結果を:return: 順運動学の結果
        """
        result = super().calculate()
        result["F"] = self.F
        return result

    def format_result(self):
        """
        計算結果をフォーマットして返す
        :return: フォーマットされた結果
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
        for key, point in self.points.items():
            self.transformed_points[key] = transformer.transform_point(point)

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
        return self.points

    def get_link_angle(self, link_index):
        """
        指定されたリンクの角度を取得する
        :param link_index: リンクのインデックス (0: B1-M1, 1: M1-X, 2: X-E, 3: E-F)
        :return: リンクの角度（ラジアン）
        """
        if link_index == 0:
            return self.theta1
        elif link_index == 1:
            return np.arctan2(self.X[1] - self.M1[1], self.X[0] - self.M1[0])
        elif link_index == 2:
            return np.arctan2(self.E[1] - self.X[1], self.E[0] - self.X[0])
        elif link_index == 3:
            return self.thetaF
        else:
            raise ValueError("Invalid link index")

    def get_link_points(self):
        """
        リンクの端点を取得する
        :return: リンクの端点のリスト
        """
        return [
            (self.B1, self.M1),
            (self.M1, self.X),
            (self.X, self.E),
            (self.E, self.F)
        ]

    def get_link_colors(self):
        """
        リンクの色を取得する
        :return: リンクの色のリスト
        """
        return ['red', 'green', 'blue', 'purple']

    def get_point_colors(self):
        """
        点の色を取得する
        :return: 点の色の辞書
        """
        return {
            'B1': 'black', 'M1': 'red', 'X': 'green',
            'M2': 'blue', 'B2': 'black', 'E': 'purple', 'F': 'orange'
        }

if __name__ == "__main__":
    # サンプルの値を設定
    b = 200
    m = 400
    e = 200
    f = 150
    B1 = (100, -100)
    B2 = (-100, -100)

    # 拡張運動学インスタンス作成
    ek = ExtendedKinematics(b, m, e, f, B1, B2)

    # モーターの角度を設定 (サンプル値)
    theta1 = -45 
    theta2 = -135 
    thetaF = -60
    ek.set_angles(theta1, theta2, thetaF)
    ek.compute_forward_kinematics()

    # 結果を表示
    result = ek.calculate()
    for key, value in result.items():
        print(f"{key}: {value}")

    # リ度を表示
    for i in range(4):
        print(f"Link {i} angle: {np.degrees(ek.get_link_angle(i)):.2f} degrees")
