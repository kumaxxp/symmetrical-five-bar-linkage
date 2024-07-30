import numpy as np
import matplotlib.pyplot as plt

def calculate_interior_angle_sum(points):
    """
    指定された点群から多角形の内角の和を計算します。
    
    Parameters:
    - points: 頂点の座標リスト。例: [[x1, y1], [x2, y2], [x3, y3], ...]
    
    Returns:
    - 内角の和（度単位）
    """
    def angle_between(p1, p2, p3):
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
        angle_sum += angle_between(p1, p2, p3)
    
    return angle_sum

def circle_intersections(center1, radius1, center2, radius2):
    """
    2つの円の交点を計算します。
    
    Parameters:
    - center1: 円1の中心座標 [x1, y1]
    - radius1: 円1の半径
    - center2: 円2の中心座標 [x2, y2]
    - radius2: 円2の半径
    
    Returns:
    - intersections: 交点の座標のリスト。交点がない場合は空リスト。
    """
    # 円の中心間の距離
    d = np.linalg.norm(np.array(center2) - np.array(center1))
    
    # 円の交差条件
    if d > radius1 + radius2 or d < abs(radius1 - radius2) or d == 0:
        return []  # 交差しない、または同一の円

    # 中心間の直線上の交点座標
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h2 = radius1**2 - a**2
    if h2 < 0:
        return []  # 交差しない

    h = np.sqrt(h2)
    x0 = center1[0] + a * (center2[0] - center1[0]) / d
    y0 = center1[1] + a * (center2[1] - center1[1]) / d
    dx = (center2[1] - center1[1]) / d
    dy = (center2[0] - center1[0]) / d
    
    # 交点の計算
    intersection1 = [x0 + h * dx, y0 - h * dy]
    intersection2 = [x0 - h * dx, y0 + h * dy]
    
    return [intersection1, intersection2]


class InverseKinematics:
    def __init__(self, Yb, l, b, m, e):
        self.Yb = Yb
        self.l = l
        self.b = b
        self.m = m
        self.e = e
        self.B1 = np.array([l/2, Yb])
        self.B2 = np.array([-l/2, Yb])
        self.E = None
    
    def set_endeffector(self, E):
        self.E = E
    
    def compute_inverse_kinematics(self):

        valid_combinations = []
        X_candidates = []

        # M1の候補を2つ計算
        M1_candidates = circle_intersections(self.B1, self.b, self.E, self.m+self.e)

        for M1 in M1_candidates:
            d = np.linalg.norm(self.E - M1)
            if d > self.e:
                # Xの候補を計算
                X_candidate = self.E + self.e * (M1 - self.E) / d
                X_candidates.append(X_candidate)

            # M2の候補を2つ計算
            M2_candidates = circle_intersections(self.B2, self.b, X_candidate, self.m)
            for M2 in M2_candidates:
                if self.is_convex([self.B1, M1, X_candidate, M2, self.B2]):
                    valid_combinations.append((M1, X_candidate, M2))
                
        self.valid_combinations = valid_combinations
    
    def calculate_M1_candidates(self):
        return circle_intersections(self.B1, self.b, self.E, self.l+self.e)
    
    def calculate_X_candidates(self, M1):
        d = np.linalg.norm(self.E - M1)
        if d > self.e:
            X = self.E + self.e * (M1 - self.E) / d

        return X
    
    def calculate_M2_candidates(self, X):
        return circle_intersections(self.B2, self.b, X, self.m)
    
    def is_convex(self, points):

        angle_sum = calculate_interior_angle_sum(points)
        n = len(points)
        expected_angle_sum = (n - 2) * 180  # 内角の和の期待値 (度)

        if abs(angle_sum - expected_angle_sum) >= 1e-6:
            return False

        def cross(x1, y1, x2, y2):
            return x1 * y2 - x2 * y1
        
        n = len(points)
        for i in range(n):
            a = points[i % n]
            b = points[(i + 1) % n]
            c = points[(i + 2) % n]
            vec_ab = [b[0] - a[0], b[1] - a[1]]
            vec_bc = [c[0] - b[0], c[1] - b[1]]
            if cross(*vec_ab, *vec_bc) > 0:
                return False
        return True


    def compute_bounding_box(self):
        # 到達可能な点の範囲を計算し、バウンディングボックスを返す
        reachable_x = [result[0] for result in self.valid_combinations]
        reachable_y = [result[1] for result in self.valid_combinations]
        x_min, x_max = min(reachable_x), max(reachable_x)
        y_min, y_max = min(reachable_y), max(reachable_y)
        return (x_min, x_max, y_min, y_max)

    def calculate_area(self):
        # 到達可能な領域の面積を計算する
        from scipy.spatial import ConvexHull
        if len(self.valid_combinations) >= 3:
            hull = ConvexHull(np.array(self.valid_combinations))
            return hull.volume
        return 0

    def extract_contours(self):
        # 到達可能領域のコンターを抽出する
        from scipy.spatial import ConvexHull
        if len(self.valid_combinations) >= 3:
            points = np.array(self.valid_combinations)
            hull = ConvexHull(points)
            return points[hull.vertices]
        return np.array([])

    def is_target_reachable(self, target_position):
        # 任意の目標位置が到達可能かどうかを判定する
        self.set_endeffector(target_position)
        self.compute_inverse_kinematics()
        return bool(self.valid_combinations)


def plot_kinematics(ik):
    
    plt.figure()
    
    # 点とリンクの描画
    if ik.valid_combinations:
        for combination in ik.valid_combinations:
            M1, X, M2 = combination
            plt.plot([ik.B1[0], M1[0]], [ik.B1[1], M1[1]], 'ro-')
            plt.plot([M1[0], X[0]], [M1[1], X[1]], 'go-')
            plt.plot([X[0], M2[0]], [X[1], M2[1]], 'bo-')
            plt.plot([M2[0], ik.B2[0]], [M2[1], ik.B2[1]], 'mo-')
            plt.plot([ik.B1[0], ik.B2[0]], [ik.B1[1], ik.B2[1]], 'k--')
            plt.plot([X[0], ik.E[0]], [X[1], ik.E[1]], 'ko-')
            
            plt.plot(ik.B1[0], ik.B1[1], 'ro')  # B1
            plt.plot(ik.B2[0], ik.B2[1], 'ro')  # B2
            plt.plot(M1[0], M1[1], 'go')  # M1
            plt.plot(X[0], X[1], 'bo')  # X
            plt.plot(M2[0], M2[1], 'mo')  # M2
            plt.plot(ik.E[0], ik.E[1], 'ko')  # E
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Inverse Kinematics')
    plt.grid()
    # x軸とy軸の表示範囲を設定
    plt.xlim(-600, 600)
    plt.ylim(-1000, 200)    

    plt.show()

if __name__ == "__main__":
    # サンプルの値を設定
    Yb = 100  # B1, B2 の Y 座標
    l = 200    # B1 から B2 までの距離
    b = 200     # B1-M1 および B2-M2 のリンク長
    m = 400    # M1-X および M2-X のリンク長
    e = 200     # X-E の距離
    
    # 逆運動学インスタンスの作成
    ik = InverseKinematics(Yb, l, b, m, e)
    E = np.array([-100,-400])
    ik.set_endeffector(E)
    ik.compute_inverse_kinematics()
    
    # 順運動学の結果をプロット
    plot_kinematics(ik)
