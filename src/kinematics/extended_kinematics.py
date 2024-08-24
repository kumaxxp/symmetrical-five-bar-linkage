import numpy as np
import math
import matplotlib.pyplot as plt
from kinematics.linkage_kinematics import ForwardKinematics
from kinematics.transformation import Transformation2D

def calculate_HardPoint(P1, P2, shift = 0):
    """
    P1-P2の直線上で、P1からshiftの距離にある点を計算する

    :param P1: 点P1の座標 (x1, y1)
    :param P2: 点P2の座標 (x2, y2)
    :param shift: P1からの距離
    :return: shiftした座標 (x, y)
    """
    x1, y1 = P1
    x2, y2 = P2

    # 方向ベクトルを計算
    dx = x2 - x1
    dy = y2 - y1

    # ベクトルの長さを計算
    length = math.sqrt(dx**2 + dy**2)

    # ベクトルを正規化（単位ベクトルにする）
    if length != 0:
        unit_dx = dx / length
        unit_dy = dy / length
    else:
        return P1  # P1とP2が同じ点の場合、P1を返す

    # shiftした座標を計算
    x = x1 + shift * unit_dx
    y = y1 + shift * unit_dy

    return (x, y)

def calculate_WirePoint(P1, P2, w, side='left'):
    """
    P1-P2の直線に直交し、P2を通る直線l上で、P2からwの距離にある点のうち、
    指定された側（左または右）に位置する点をW2として計算する。

    :param P1: 点P1の座標 (x1, y1)
    :param P2: 点P2の座標 (x2, y2)
    :param w: P2からW2までの距離
    :param side: 'left'または'right'を指定。デフォルトは'left'
    :return: W2の座標 (x, y)
    """
    # ステップ1: ベクトルP1P2の定義
    x1, y1 = P1
    x2, y2 = P2
    v = np.array([x2 - x1, y2 - y1])

    # ステップ2: P1P2に直交するベクトルの計算
    v_perp = np.array([-v[1], v[0]])
    
    # ステップ3: v_perpの正規化
    v_perp_norm = v_perp / np.linalg.norm(v_perp)

    # ステップ4: W2の候補となる2点を計算
    W2_candidate1 = P2 + w * v_perp_norm
    W2_candidate2 = P2 - w * v_perp_norm

    # ステップ5: 各候補点がP1-P2ベクトルの左側にあるか右側にあるかを判定
    cross_product1 = np.cross(v, W2_candidate1 - np.array(P1))
    cross_product2 = np.cross(v, W2_candidate2 - np.array(P1))

    # ステップ6: 指定された側に応じてW2を選択
    if side == 'left':
        W2 = W2_candidate1 if cross_product1 > 0 else W2_candidate2
    else:  # 'right'
        W2 = W2_candidate2 if cross_product1 < 0 else W2_candidate2

    return tuple(W2)

def calculate_P3(P1, P2, L, theta):
    """
    P1からP2へのベクトルとP1からP3へのベクトルを用いてP3の位置を計算する関数
    :param P1: P1の座標 (x1, y1)
    :param P2: P2の座標 (x2, y2)
    :param L: P1からP3へのベクトルの長さ
    :param theta: P1からP2へのベクトルとP1からP3へのベクトルとの間の角度 (度)
    :return: P3の座標 (x3, y3)
    """
    theta = float(theta)  # thetaを確実に浮動小数点数に変換
        
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
    def __init__(self, b, m, e, f, B1, B2, W1, W2, w):
        """
        初期化メソッド
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        :param f: 追加リンクの長さ
        :param B1: B1の座標 (x, y)
        :param B2: B2の座標 (x, y)
        :param W1: W1の座標 (x, y)
        :param W2: W2の座標 (x, y)
        :param w: リンクとワイヤーの距離
        """
        super().__init__(b, m, e, B1, B2)
        self.f = f
        self.F = None
        self.thetaF = 0
        self.points = {}
        transformed_points = {}
        self.rotated_points = {}
        self.B1 = B1
        self.B2 = B2
        self.W1 = W1
        self.W2 = W2
        self.w  = w
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
        順運動学を計算し、F, W1, W2の位置を含める
        """
        super().compute_forward_kinematics()
        self.F = self.calculate_F()
        self.W11 = calculate_WirePoint(self.B1, self.M1, self.w, 'left')
        self.W21 = calculate_WirePoint(self.B2, self.M2, self.w, 'right')

        self.W12 = calculate_WirePoint(self.X, self.M1, self.w, 'right')
        self.W22 = calculate_WirePoint(self.X, self.M2, self.w, 'left')

        self.W31 = calculate_HardPoint(self.X, self.M2, 20)
        self.W32 = calculate_HardPoint(self.E, self.F, -20)

        self.W41 = calculate_WirePoint(self.E, self.X, self.w, 'right')
        self.W42 = calculate_HardPoint(self.E, self.F, 20)

        self.points = self.format_result()


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
    
    def calculate_rotated_points(self, pointSt1, pointEd1, pointSt2, pointEd2):
        angle_flower = math.degrees(self.angle_between_vectors(pointSt1, pointEd1, pointSt2, pointEd2))
        #transformer = Transformation2D(origin=pointSt1, angle=angle_flower, translation=-np.array(pointSt1))
        transformer = Transformation2D(origin=pointSt1, angle=angle_flower, translation=-np.array(pointSt1))
        self.rotated_points = {key: transformer.transform_point(value) for key, value in self.points.items()}

    def calculate_translate_points(self, point_translate):
        # 回転後にポイントをシフトさせる
        self.rotated_points = {key: point_translate + value for key, value in self.rotated_points.items()}

    def get_rotated_points(self):
        return self.rotated_points
    
    def get_points(self):
        return self.points

    @staticmethod
    def angle_between_vectors(p1, p2, p3, p4):
        v1 = np.array(p2) - np.array(p1)
        v2 = np.array(p4) - np.array(p3)
        return math.atan2(np.cross(v1, v2), np.dot(v1, v2))

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
            "X2": self.X2,
            "W1": self.W1,
            "W2": self.W2,
            "W11": self.W11,
            "W12": self.W12,
            "W21": self.W21,
            "W22": self.W22,
            "W31": self.W31,
            "W32": self.W32,
            "W41": self.W41,
            "W42": self.W42

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

    def get_link_angle(self, vect):
        """
        指定された頂点から次の頂点への角度を取得する
        :param vect: 開始頂点の名前 ('B1', 'B2', 'E')
        :return: リンクの角度（ラジアン）
        """
        link_order = ['B1', 'B2', 'E']
        
        if vect not in link_order[:-1]:
            raise ValueError("Invalid vect point")
                
        if vect == 'B1':
            return self.theta1
        elif vect == 'B2':
            return self.theta2
        elif vect == 'E':
            return self.thetaF
        else:
            raise ValueError("Invalid vect point")


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

