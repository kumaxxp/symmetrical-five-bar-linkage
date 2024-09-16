import numpy as np
import math
import matplotlib.pyplot as plt
from kinematics.linkage_kinematics import ForwardKinematics
from kinematics.transformation import Transformation2D

import math
from typing import Tuple, Optional, Union

def calculate_perpendicular_points(
    G: Union[Tuple[float, float], np.ndarray],
    E: Union[Tuple[float, float], np.ndarray],
    h: float
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    GからEへの直線と直角で交わり、Eを交点とする直線上にあり、Eからの距離がhである2つの点を計算する

    :param G: Gの座標 (x, y)
    :param E: Eの座標 (x, y)
    :param h: Eからの距離
    :return: hだけ離れた2つの点の座標 ((x1, y1), (x2, y2))
    """
    # GからEへのベクトルを計算
    GE = np.array(E) - np.array(G)

    # ベクトルの長さを計算
    length = np.linalg.norm(GE)

    # ベクトルを正規化（単位ベクトルにする）
    if length != 0:
        unit_GE = GE / length
    else:
        return (E, E)  # GとEが同じ点の場合、Eを返す

    # 直角なベクトルを計算
    perpendicular_vector1 = np.array([-unit_GE[1], unit_GE[0]])
    perpendicular_vector2 = np.array([unit_GE[1], -unit_GE[0]])

    # hだけ離れた2つの点を計算
    point1 = (E[0] + h * perpendicular_vector1[0], E[1] + h * perpendicular_vector1[1])
    point2 = (E[0] + h * perpendicular_vector2[0], E[1] + h * perpendicular_vector2[1])

    return (point1, point2)

def find_farthest_combination(
    H1: Tuple[float, float],
    H2: Tuple[float, float],
    I1: Tuple[float, float],
    I2: Tuple[float, float]
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    H1からI1およびI2との距離、H2からI1およびI2との距離の組み合わせをチェックし、一番遠い組み合わせを計算する

    :param H1: H1の座標 (x, y)
    :param H2: H2の座標 (x, y)
    :param I1: I1の座標 (x, y)
    :param I2: I2の座標 (x, y)
    :return: 一番遠い組み合わせの座標 ((H, I), (H, I))
    """
    def distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return np.linalg.norm(np.array(p1) - np.array(p2))

    combinations = [
        (H1, I1),
        (H1, I2),
        (H2, I1),
        (H2, I2)
    ]

    max_distance = 0
    farthest_combination = (H1, I1)

    for (H, I) in combinations:
        dist = distance(H, I)
        if dist > max_distance:
            max_distance = dist
            farthest_combination = (H, I)

    return farthest_combination

def find_circle_centers(
    P1: Tuple[float, float],
    P2: Tuple[float, float],
    P3: Tuple[float, float],
    r: float
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    2つの点P1, P2と半径rから、円の中心点C1, C2を計算して返します。
    
    Args:
        P1 (Tuple[float, float]): 点P1の座標 (x1, y1)
        P2 (Tuple[float, float]): 点P2の座標 (x2, y2)
        r (float): 円の半径
    
    Returns:
        Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
            中心点C1とC2の座標を含むタプル。条件を満たさない場合はNoneを返す。
    
    Raises:
        ValueError: 半径rが正でない場合、または2点間の距離が2rを超える場合に発生。
    """
    x1, y1 = P1
    x2, y2 = P2
    x3, y3 = P3

    # 2点間の距離dを計算
    dx = x2 - x1
    dy = y2 - y1
    d_sq = dx**2 + dy**2
    d = math.sqrt(d_sq)

    # 半径rが正であることを確認
    if r <= 0:
        raise ValueError("半径rは正の値でなければなりません。")

    # 2点間の距離が2rを超える場合、条件を満たす円は存在しない
    if d > 2 * r:
        raise ValueError("2点間の距離が2rを超えているため、条件を満たす円は存在しません。")

    # 中点Mの座標を計算
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    # 中点から中心までの距離hを計算
    # h^2 = r^2 - (d/2)^2
    h_sq = r**2 - (d / 2)**2
    # 数値誤差のため、h_sqが微小な負数になっていないか確認
    if h_sq < 0:
        h_sq = 0
    h = math.sqrt(h_sq)

    # 単位垂直ベクトルを計算
    if d == 0:
        # P1とP2が同一の場合、無限に多くの中心が存在する
        raise ValueError("点P1とP2が同一であるため、中心を一意に決定できません。")
    
    # 垂直方向の単位ベクトル (−dy/d, dx/d)
    ux = -dy / d
    uy = dx / d

    # 中心点C1とC2を計算
    C1 = (mx + h * ux, my + h * uy)
    C2 = (mx - h * ux, my - h * uy)

    # C1,C2のうち、P3に距離が近いものを選択
    # P3からC1, C2への距離を計算
    dist_C1 = math.sqrt((C1[0] - x3)**2 + (C1[1] - y3)**2)
    dist_C2 = math.sqrt((C2[0] - x3)**2 + (C2[1] - y3)**2)

    # P3に距離が近い中心点を返す
    if dist_C1 < dist_C2:
        return C1
    else:
        return C2 


def calculateHardPoint(P1, P2, shift = 0):
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

def calculateWirePoint(P1, P2, w, side='left'):
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

def calculateP3(P1, P2, L, theta):
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

        self.Rfe = 500  # F-Eの足の扇型の半径
        self.G = None
        self.H = None
        self.I = None
        self.h = 100
        self.i = 200

    def setAngles(self, theta1, theta2, thetaF):
        """
        モーターの角度設定
        :param theta1: B1-M1 の角度 (度)
        :param theta2: B2-M2 の角度 (度)
        :param thetaF: X-E-F の角度 (度)
        """
        super().setAngles(theta1, theta2)
        self.thetaF = thetaF
        self.theta1 = theta1
        self.theta2 = theta2

    def computeForwardKinematics(self):
        """
        順運動学を計算し、F, W1, W2の位置を含める
        """
        super().computeForwardKinematics()
        self.F = self.calculateF()

        self.G = find_circle_centers(self.E, self.F, self.B1, self.Rfe)     # 点EFを始点終点とする円弧を描く
        H1,H2 = calculate_perpendicular_points(self.G, self.E, self.h)      # 円弧から延びる直線を描き、つま先とかかとにする
        I1,I2 = calculate_perpendicular_points(self.G, self.F, self.i)      #  点E,Fから延びる接線を計算し、点H,Iを求める
        self.H, self.I = find_farthest_combination(H1,H2,I1,I2)             #  H,Iの距離が最も遠い組み合わせを選択する
        
        self.W11 = calculateWirePoint(self.B1, self.M1, self.w, 'left')
        self.W21 = calculateWirePoint(self.B2, self.M2, self.w, 'right')

        self.W12 = calculateWirePoint(self.X, self.M1, self.w, 'right')
        self.W22 = calculateWirePoint(self.X, self.M2, self.w, 'left')

        self.W31 = calculateHardPoint(self.X, self.M2, 20)
        self.W32 = calculateHardPoint(self.E, self.F, -20)

        self.W41 = calculateWirePoint(self.E, self.X, self.w, 'right')
        self.W42 = calculateHardPoint(self.E, self.F, 20)

        self.points = self.formatResult()


    def calculateF(self):
        """
        EからFへの位置を計算
        """
        if self.E is None or self.X is None:
            raise ValueError("EまたはX位置が計算されていません。")

        return calculateP3(self.E, self.X, self.f, self.thetaF)

    def calculate(self):
        """
        順運動学の計算を行い、結果を:return: 順運動学の結果
        """
        result = super().calculate()
        result["F"] = self.F
        return result
    
    def calculateRotatedPoints(self, pointSt1, pointEd1, pointSt2, pointEd2):
        angle_flower = math.degrees(self.angleBetweenVectors(pointSt1, pointEd1, pointSt2, pointEd2))
        #transformer = Transformation2D(origin=pointSt1, angle=angle_flower, translation=-np.array(pointSt1))
        transformer = Transformation2D(origin=pointSt1, angle=angle_flower, translation=-np.array(pointSt1))
        self.rotated_points = {key: transformer.transformPoint(value) for key, value in self.points.items()}

    def calculateTranslatePoints(self, point_translate):
        # 回転後にポイントをシフトさせる
        self.rotated_points = {key: point_translate + value for key, value in self.rotated_points.items()}

    def getRotatedPoints(self):
        return self.rotated_points
    
    def getPoints(self):
        return self.points

    @staticmethod
    def angleBetweenVectors(p1, p2, p3, p4):
        v1 = np.array(p2) - np.array(p1)
        v2 = np.array(p4) - np.array(p3)
        return math.atan2(np.cross(v1, v2), np.dot(v1, v2))

    def formatResult(self):
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
            "W42": self.W42,
            "G":   self.G,
            "H":   self.H,
            "I":   self.I
        }
    
    def getLengthInfo(self, cmd = 'default'):
        """
        ポイントにむずびつけられたワイヤーの長さを返す
        :return: フォーマットされた結果
        """


        # W1とW11の二点間の距離
        lW1_W11  = np.linalg.norm(np.array(self.W1) - np.array(self.W11))
        lW11_W12 = np.linalg.norm(np.array(self.W11) - np.array(self.W12))
        self.diff_W1 = lW1_W11 + lW11_W12
        
        lW2_W21  = np.linalg.norm(np.array(self.W2) - np.array(self.W21))
        lW21_W22 = np.linalg.norm(np.array(self.W21)- np.array(self.W22))
        self.diff_W2 = lW2_W21 + lW21_W22

        self.diff_W3 = np.linalg.norm(np.array(self.W31) - np.array(self.W32))
        self.diff_W4 = np.linalg.norm(np.array(self.W41) - np.array(self.W42))

        if cmd == 'default': # ワイヤーの長さ

            return {
                "W1-W11-W12": self.diff_W1,
                "W2-W21-W22": self.diff_W2,
                "W31-W32": self.diff_W3,
                "W41-W42": self.diff_W4
            }

        elif cmd == 'spring': # リンクとワイヤーの差。バネ成分
            return {
                "W1-W11-W12": self.diff_W1 - self.b,
                "W2-W21-W22": self.diff_W2 - self.b,
                "W31-W32": self.diff_W3 - self.e,
                "W41-W42": self.diff_W4 - self.e
            }
        
        elif cmd == 'diff':
            return {
                "W1-W11-W12": lW11_W12,
                "W2-W21-W22": lW21_W22,
                "W31-W32": 0,
                "W41-W42": 0
            }

        elif cmd == 'link': # ワイヤーと平行なリンクの長さ
            return {
                "W1-W11-W12": self.b,
                "W2-W21-W22": self.b,
                "W31-W32": self.e,
                "W41-W42": self.e
            }

    def applyTransformation(self, transformer):
        """
        座標変換を適用する
        :param transformer: 変換器オブジェクト
        """
        for key, point in self.points.items():
            self.transformed_points[key] = transformer.transform_point(point)

    def getTransformedPoints(self):
        """
        変換された座標を取得する
        :return: 変換された座標
        """
        return self.transformed_points

    def getOriginalPoints(self):
        """
        元の座標を取得する
        :return: 元の座標
        """
        return self.points

    def getLinkAngle(self, vect):
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


    def getLinkPoints(self):
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

    def getLinkColors(self):
        """
        リンクの色を取得する
        :return: リンクの色のリスト
        """
        return ['red', 'green', 'blue', 'purple']

    def getPointColors(self):
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
    ek.setAngles(theta1, theta2, thetaF)
    ek.computeForwardKinematics()

    # 結果を表示
    result = ek.calculate()
    for key, value in result.items():
        print(f"{key}: {value}")

