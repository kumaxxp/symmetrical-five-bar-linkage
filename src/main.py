import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from extended_kinematics import ExtendedKinematics

class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title('Kinematics Visualization')
        self.geometry('1200x1000')  # ウィンドウサイズを拡大

        # 初期角度の設定
        self.initial_theta1 = -45
        self.initial_theta2 = -135
        self.initial_thetaF = -60

        # メインフレームの作成
        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # グラフ用フレームの作成（左側に2つのグラフを配置）
        self.graph_frame = ttk.Frame(self.main_frame)
        self.graph_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 1つ目のグラフのフレーム
        self.figure1, self.ax1 = plt.subplots()
        self.canvas1 = FigureCanvasTkAgg(self.figure1, self.graph_frame)
        self.canvas1.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 2つ目のグラフのフレーム
        self.figure2, self.ax2 = plt.subplots()
        self.canvas2 = FigureCanvasTkAgg(self.figure2, self.graph_frame)
        self.canvas2.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        # スライダ用フレームの作成（右側に配置）
        self.slider_frame = ttk.Frame(self.main_frame)
        self.slider_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # スライダとその値を表示するラベルの作成
        self.slider_theta1 = self.create_slider_with_label('Theta1', self.initial_theta1, self.slider_frame)
        self.slider_theta2 = self.create_slider_with_label('Theta2', self.initial_theta2, self.slider_frame)
        self.slider_thetaF = self.create_slider_with_label('ThetaF', self.initial_thetaF, self.slider_frame)

        self.ek = self.initialize_kinematics()
        self.update_plot()

    def create_slider_with_label(self, label_text, initial_value, parent_frame):
        # ラベルとスライダ用のフレーム作成
        frame = ttk.Frame(parent_frame)
        frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        # ラベルの作成
        label = ttk.Label(frame, text=label_text)
        label.pack(side=tk.LEFT)

        # 現在の値を表示するラベル
        value_label = ttk.Label(frame, text=str(initial_value))
        value_label.pack(side=tk.RIGHT)

        # スライダの作成
        slider = ttk.Scale(frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=lambda v: self.update_slider_value(v, value_label))
        slider.set(initial_value)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # スライダを返す
        return slider

    def update_slider_value(self, value, value_label):
        # スライダの値をラベルに表示
        value_label.config(text=str(int(float(value))))
        self.update_plot()

    def initialize_kinematics(self):
        # 順運動学の初期化
        Yb = 100
        l = 200
        b = 200
        m = 400
        e = 200
        f = 150
        ek = ExtendedKinematics(Yb, l, b, m, e, f)
        ek.set_angles(self.initial_theta1, self.initial_theta2, self.initial_thetaF)
        ek.compute_forward_kinematics()
        return ek

    def update_plot(self, event=None):
        try:
            theta1 = self.slider_theta1.get()
            theta2 = self.slider_theta2.get()
            thetaF = self.slider_thetaF.get()

            self.ek.set_angles(theta1, theta2, thetaF)
            self.ek.compute_forward_kinematics()

            self.plot_extended_kinematics(self.ax1, self.canvas1)  # 1つ目のグラフにプロット
            self.plot_extended_kinematics(self.ax2, self.canvas2)  # 2つ目のグラフにプロット
        except Exception as e:
            print(f"エラーが発生しました: {e}")

    def plot_line(self, ax, point1, point2, color, label):
        """2点間の線をプロットするヘルパー関数"""
        if point1 is not None and point2 is not None:
            ax.plot([point1[0], point2[0]], [point1[1], point2[1]], f'{color}-o', label=label)

    def plot_extended_kinematics(self, ax, canvas):
        ax.clear()

        result = self.ek.calculate()
        B1, M1, X, M2, B2, E, F = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["F"]

        # 線をプロット
        self.plot_line(ax, B1, M1, 'r', 'B1-M1')
        self.plot_line(ax, M1, X, 'b', 'M1-X')
        self.plot_line(ax, X, M2, 'g', 'X-M2')
        self.plot_line(ax, M2, B2, 'y', 'M2-B2')
        self.plot_line(ax, X, E, 'm', 'X-E')
        self.plot_line(ax, E, F, 'c', 'E-F')

        # ポイントをプロット
        for point, color, label in zip([B1, M1, X, M2, B2, E, F],
                                       ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan'],
                                       ['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F']):
            if point is not None:
                ax.scatter(*point, color=color, zorder=5)
                ax.text(point[0], point[1], label, fontsize=12, ha='right')

        ax.set_xlim(-600, 600)
        ax.set_ylim(-1000, 200)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Extended Kinematics Visualization')
        ax.grid(True)
        ax.legend()
        canvas.draw()

    def destroy(self):
        # Matplotlibのクリーンアップ
        self.canvas1.get_tk_widget().destroy()
        self.canvas2.get_tk_widget().destroy()
        plt.close(self.figure1)  # Figureを閉じる
        plt.close(self.figure2)  # Figureを閉じる
        super().destroy()  # Tkinterのdestroyを呼び出す

if __name__ == '__main__':
    app = KinematicsApp()
    app.mainloop()
