import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from linkage_kinematics import ForwardKinematics
from extended_kinematics import ExtendedKinematics

def plot_extended_kinematics(ek):
    result = ek.calculate()
    B1, M1, X, M2, B2, E, F = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["F"]

    plt.clf()  # 現在のプロットをクリア
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

    for point, color, label in zip([B1, M1, X, M2, B2, E, F],
                                    ['red', 'red', 'blue', 'green', 'green', 'magenta','cyan'],
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
    plt.draw()

def update(val):
    theta1 = slider_theta1.val
    theta2 = slider_theta2.val
    thetaF = slider_thetaF.val
    ek.set_angles(theta1, theta2, thetaF)
    ek.compute_forward_kinematics()
    plot_extended_kinematics(ek)

if __name__ == "__main__":
    Yb = 100
    l = 200
    b = 200
    m = 400
    e = 200
    f = 150

    ek = ExtendedKinematics(Yb, l, b, m, e, f)

    # 初期角度設定
    initial_theta1 = -45
    initial_theta2 = -135
    initial_thetaF = -60    
    ek.set_angles(initial_theta1, initial_theta2, initial_thetaF)
    ek.compute_forward_kinematics()

    # プロットの初期化
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)
    plot_extended_kinematics(ek)

    # スライダーの設定
    ax_theta1 = plt.axes([0.1, 0.1, 0.65, 0.03])
    ax_theta2 = plt.axes([0.1, 0.15, 0.65, 0.03])
    ax_thetaF = plt.axes([0.1, 0.2,0.65, 0.03])

    slider_theta1 = Slider(ax_theta1, 'Theta1', -180, 180, valinit=initial_theta1)
    slider_theta2 = Slider(ax_theta2, 'Theta2', -180, 180, valinit=initial_theta2)
    slider_thetaF = Slider(ax_thetaF, 'ThetaF', -180, 180, valinit=initial_thetaF)

    slider_theta1.on_changed(update)
    slider_theta2.on_changed(update)
    slider_thetaF.on_changed(update)

    plt.show()
