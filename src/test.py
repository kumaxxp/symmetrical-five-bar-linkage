import numpy as np

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

# 使用例
if __name__ == "__main__":
    P1 = (1, 2)  # P1の座標
    P2 = (4, 6)  # P2の座標
    L = 5        # P1からP3へのベクトルの長さ
    theta = 30   # P1からP2へのベクトルとP1からP3へのベクトルとの間の角度 ()

    P3 = calculate_P3(P1, P2, L, theta)
    print(f"P3の座標: {P3}")
