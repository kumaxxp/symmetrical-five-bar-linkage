import numpy as np
import matplotlib.pyplot as plt

def calculateInteriorAngleSum(points):
    """
    指定された点群から多角形の内角の和を計算します。
    
    Parameters:
    - points: 頂点の座標リスト。例: [[x1, y1], [x2, y2], [x3, y3], ...]
    
    Returns:
    - 内角の和（度単位）
    """
    def angleBetween(p1, p2, p3):
        """
        3点 p1, p2, p3 から角度を計算します。
        """
        # ベクトルを計算
        v1 = np.array(p1) - np.array(p2)
        v2 = np.array(p3) - np.array(p2)
        
        # 内積とノルム
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # 角度の計算
        cos_theta = dot_product / (norm_v1 * norm_v2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # 数値誤差対策
        angle_rad = np.arccos(cos_theta)
        return np.degrees(angle_rad)

    n = len(points)
    if n < 3:
        raise ValueError("少なくとも3点必要です。")

    angle_sum = 0.0
    for i in range(n):
        p1 = points[i]
        p2 = points[(i + 1) % n]
        p3 = points[(i + 2) % n]
        angle_sum += angleBetween(p1, p2, p3)
    
    return angle_sum

class ForwardKinematics:
    def __init__(self, b, m, e, B1, B2):
        """
        初期化メソッド
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        """
        self.b = b
        self.m = m
        self.e = e

        # B1, B2 の座標を設定
        self.B1 = B1
        self.B2 = B2
        
        self.M1 = None
        self.M2 = None
        self.X = None
        self.E = None

        self.X1 = None
        self.X2 = None

        self.theta1 = None
        self.theta2 = None

    def setAngles(self, theta1, theta2):
        """
        モーターの角度を設定
        :param theta1: B1-M1 の角度 (度)
        :param theta2: B2-M2 の角度 (度)
        """

        self.theta1 = theta1
        self.theta2 = theta2
    
    def computeForwardKinematics(self):
        # 角度をラジアンに変換
        theta1_rad = np.radians(self.theta1)
        theta2_rad = np.radians(self.theta2)
        
        try:
            # M1, M2 の座標を計算
            self.M1 = self.B1 + np.array([self.b * np.cos(theta1_rad), self.b * np.sin(theta1_rad)])
            self.M2 = self.B2 + np.array([self.b * np.cos(theta2_rad), self.b * np.sin(theta2_rad)])
        
            # X の座標を計算
            self.X = self.calculateX()
        
            # エンドエフェクタ E の座標を計算
            self.E = self.calculateE()

        except ValueError as e:
            print(f"Error in forward kinematics: {e}")
            self.X = None
            self.E = None

    def calculateE(self):
        # 点Eの計算
        X = self.X
        M1 = self.M1
        
        # 直線 M1-X の方程式
        dx = X[0] - M1[0]
        dy = X[1] - M1[1]
        norm = np.sqrt(dx**2 + dy**2)
        
        # 点Eの計算
        E_x = X[0] + self.e * dx / norm
        E_y = X[1] + self.e * dy / norm
        
        return np.array([E_x, E_y])

    def calculateX(self):
        """
        点 X の座標を計算
        :return: X の座標 (x, y)
        """
        # M1-X と M2-X が同じ長さの円の交点を計算
        d = np.linalg.norm(self.M1 - self.M2)
        if d > 2 * self.m:
            raise ValueError("No valid intersection between circles. Increase link length m.")
        
        # 円の半径
        r1, r2 = self.m, self.m
        x1, y1 = self.M1
        x2, y2 = self.M2

        # 円の交点を計算
        X1,X2 = self.calculateCircleIntersection(x1, y1, r1, x2, y2, r2)
    
        self.X1 = X1
        self.X2 = X2

        # 両方のX点で内角と凸形状チェックを行う
        valid_X = None
        for X_candidate in [X1, X2]:
            points = [self.B1, self.M1, X_candidate, self.M2, self.B2]
            if self.isConvex(points) :
                valid_X = X_candidate
                break
        
        if valid_X is None:
            raise ValueError("No valid X point found that satisfies the convex pentagon condition.")
        
        return valid_X


    def calculateCircleIntersection(self, x1, y1, r1, x2, y2, r2):
        """
    2つの円の交点を計算する
        :param x1: 円1の中心のx座標
        :param y1: 円1の中心のy座標
        :param r1: 円1の半径
        :param x2: 円2の中心のx座標
        :param y2: 円2の中心のy座標
        :param r2: 円2の半径
        :return: 交点の座標のリスト
        """
        # 中心間距離
        d = np.linalg.norm([x2 - x1, y2 - y1])
        
        if d > r1 + r2:
            raise ValueError("No intersection: circles are too far apart.")
        if d < abs(r1 - r2):
            raise ValueError("No intersection: one circle is contained within the other.")
        
        # 中心間の中点
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = np.sqrt(r1**2 - a**2)
        x0 = x1 + a * (x2 - x1) / d
        y0 = y1 + a * (y2 - y1) / d

        # 交点の算
        intersection1 = np.array([x0 + h * (y2 - y1) / d, y0 - h * (x2 - x1) / d])
        intersection2 = np.array([x0 - h * (y2 - y1) / d, y0 + h * (x2 - x1) / d])

        return intersection1, intersection2

    def cross(self, x1,y1,x2,y2):
        return x1*y2 - x2*y1

    def isConvex(self, points):
        """
        凸形状かどうかを判定する
        :param points: 頂点の座標リスト
        :return: 凸形状であれば True, そうでなければ False
        """
        angle_sum = calculateInteriorAngleSum(points)
        n = len(points)
        expected_angle_sum = (n - 2) * 180  # 内角の和の期待値 (度)

        if abs(angle_sum - expected_angle_sum) >= 1e-6:
            return False

        n = len(points)
        for i in range(n):
            a = points[i % n]
            b = points[(i + 1) % n]
            c = points[(i + 2) % n]
            vec_ab = [b[0] - a[0], b[1] - a[1]]
            vec_bc = [c[0] - b[0], c[1] - b[1]]
            if self.cross(*vec_ab, *vec_bc) > 0:
                return False
        return True

    def calculate(self):
        """
        順運動学の計算を行い、結果を返す
        :return: 順運動学の結果
        """
        return self.formatResult()

    def formatResult(self):
        return {
            "B1": self.B1,
            "M1": self.M1,
            "X": self.X,
            "M2": self.M2,
            "B2": self.B2,
            "E": self.E,
            "X1": self.X1,
            "X2": self.X2
        }

def plotKinematics(fk):
    """
    順運動学の結果をプロットする
    :param fk: ForwardKinematics インスタンス
    """
    result = fk.calculate()

    if isinstance(result, str):
        #print(result)
        return

    B1, M1, X, M2, B2, E, X1, X2 = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["X1"], result["X2"]

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
    
    if B1 is not None:
        plt.scatter(*B1, color='red', zorder=5)
    if M1 is not None:
        plt.scatter(*M1, color='red', zorder=5)
    if X is not None:
        plt.scatter(*X, color='blue', zorder=5)
    if M2 is not None:
        plt.scatter(*M2, color='green', zorder=5)
    if B2 is not None:
        plt.scatter(*B2, color='green', zorder=5)
    if E is not None:
        plt.scatter(*E, color='magenta', zorder=5)
    
    if B1 is not None:
        plt.text(B1[0], B1[1], 'B1', fontsize=12, ha='right')
    if M1 is not None:
        plt.text(M1[0], M1[1], 'M1', fontsize=12, ha='right')
    if X is not None:
        plt.text(X[0], X[1], 'X', fontsize=12, ha='right')
    if M2 is not None:
        plt.text(M2[0], M2[1], 'M2', fontsize=12, ha='right')
    if B2 is not None:
        plt.text(B2[0], B2[1], 'B2', fontsize=12, ha='right')
    if E is not None:
        plt.text(E[0], E[1], 'E', fontsize=12, ha='right')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Forward Kinematics Visualization')
    plt.grid(True)
    plt.legend()
    # x軸とy軸の表示範囲を設定
    plt.xlim(-600, 600)
    plt.ylim(-1000, 200)    
#    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    b = 200     # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200     # X-E の距離
    B1 = (100, -100)
    B2 = (-100, -100)
    
    # 順運動学インスタンスの作成
    fk = ForwardKinematics(b, m, e, B1, B2)
    
    # モーターの角度を設定 (サンプル値)
    theta1 = -45
    theta2 = -135
    fk.setAngles(theta1, theta2)
    fk.computeForwardKinematics()
    
    # 順運動学の結果をプロット
    plotKinematics(fk)
