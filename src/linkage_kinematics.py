import numpy as np
import matplotlib.pyplot as plt

class ForwardKinematics:
    def __init__(self, Yb, l, b, m, e):
        """
        初期化メソッド
        :param Yb: B1, B2 の Y 座標
        :param l: B1 から B2 までの距離
        :param b: B1-M1 および B2-M2 のリンク長
        :param m: M1-X および M2-X のリンク長
        :param e: X-E の距離
        """
        self.Yb = Yb
        self.l = l
        self.b = b
        self.m = m
        self.e = e

        # B1, B2 の座標を設定
        self.B1 = np.array([self.l / 2, self.Yb])
        self.B2 = np.array([-self.l / 2, self.Yb])
        
        self.M1 = None
        self.M2 = None
        self.X = None
        self.E = None

        self.X1 = None
        self.X2 = None

    def set_angles(self, theta1, theta2):
        """
        モーターの角度を設定
        :param theta1: B1-M1 の角度 (度)
        :param theta2: B2-M2 の角度 (度)
        """
        # 角度をラジアンに変換
        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)
        
        try:
            # M1, M2 の座標を計算
            self.M1 = self.B1 + np.array([self.b * np.cos(theta1_rad), self.b * np.sin(theta1_rad)])
            self.M2 = self.B2 + np.array([self.b * np.cos(theta2_rad), self.b * np.sin(theta2_rad)])
        
            # X の座標を計算
            self.X = self.calculate_X()
        
            # エンドエフェクタ E の座標を計算
            self.E = self.X + np.array([self.e, 0])

        except ValueError as e:
            print(f"Error: {e}")
            self.X = None
            self.E = None

    def calculate_X(self):
        """
        点 X の座標を計算
        :return: X の座標 (x, y)
        """
        # M1-X と M2-X が同じ長さの円の交点を計算
        d = np.linalg.norm(self.M1 - self.M2)
        if d > 2 * self.m:
            raise ValueError("No valid intersection between circles. Increase link length m.")
        
        r1, r2 = self.m, self.m
        x1, y1 = self.M1
        x2, y2 = self.M2

        # 中心間距離
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = np.sqrt(r1**2 - a**2)
        x0 = x1 + a * (x2 - x1) / d
        y0 = y1 + a * (y2 - y1) / d
        
        # X の座標を2点計算
        X1 = np.array([x0 + h * (y2 - y1) / d, y0 - h * (x2 - x1) / d])
        X2 = np.array([x0 - h * (y2 - y1) / d, y0 + h * (x2 - x1) / d])

        self.X1 = X1
        self.X2 = X2

        print(X1)
        print(X2)
        
        # 両方のX点で内角と凸形状チェックを行う
        valid_X = None
        for X_candidate in [X1, X2]:
            self.X = X_candidate

            points = [self.B1, self.M1, self.X, self.M2, self.B2]
            if self.is_convex(5, points) :
                valid_X = X_candidate
                break
        
        if valid_X is None:
            raise ValueError("No valid X point found that satisfies the convex pentagon condition.")
        
        return valid_X

    def cross(self, x1,y1,x2,y2):
        return x1*y2 - x2*y1

    def is_convex(self, n, vertexes):
        #n多角形の判定
        flg = True #入力された図形が凸ならTrue/凸でないならFalse
        for i in range(n):
            #3頂点を時計回りに参照
            a = vertexes[i%n]
            b = vertexes[(i+1)%n]
            c = vertexes[(i+2)%n]

            #ベクトルab, bcを計算
            vec_ab = [b[0]-a[0], b[1]-a[1]]
            vec_bc = [c[0]-b[0], c[1]-b[1]]

            #外積が
            if self.cross(*vec_ab, *vec_bc)>0:
                flg = False
                break
        #flg=True の場合は凸多角形、Falseは凸でない多角形
        return flg

    def calculate(self):
        """
        順運動学の計算を行い、結果を返す
        :return: 順運動学の結果またはエラーメッセージ
        """

        # Calculate the final result (example placeholder)
        result = {
            "B1": self.B1,
            "M1": self.M1,
            "X": self.X,
            "M2": self.M2,
            "B2": self.B2,
            "E": self.E,
            "X1": self.X1,
            "X2": self.X2
        }
        
        return result

def plot_kinematics(fk):
    """
    順運動学の結果をプロットする
    :param fk: ForwardKinematics インスタンス
    """
    result = fk.calculate()

    if isinstance(result, str):
        print(result)
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
    plt.xlim(-400, 400)
    plt.ylim(-400, 400)    
#    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200     # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 50     # X-E の距離
    
    # 順運動学インスタンスの作成
    fk = ForwardKinematics(Yb, l, b, m, e)
    
    # モーターの角度を設定 (サンプル値)
    theta1 = -45
    theta2 = -135
    fk.set_angles(theta1, theta2)
    
    # 順運動学の結果をプロット
    plot_kinematics(fk)
