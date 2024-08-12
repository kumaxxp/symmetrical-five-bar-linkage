import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from extended_kinematics import ExtendedKinematics
from transformation import Transformation2D

def angle_between_vectors(v1_start, v1_end, v2_start, v2_end):
    v1 = np.array(v1_end) - np.array(v1_start)
    v2 = np.array(v2_end) - np.array(v2_start)
    angle_rad = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
    angle_deg = np.degrees(angle_rad)
    if angle_deg < -180:
        angle_deg += 360
    elif angle_deg > 180:
        angle_deg -= 360
    return angle_deg

class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title('Kinematics Visualization')
        self.geometry('1400x1000')  # ウィンドウサイズを拡大
        self.resizable(False, True)  # 横方向のリサイズを禁止、縦方向は許可


        # 初期角度の設定
        self.initial_angles = {
            'left': {'theta1': -45, 'theta2': -135, 'thetaF': -50},
            'right': {'theta1': -45, 'theta2': -135, 'thetaF': -50}
        }

        # メインフレームの作成
        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # グラフ用フレームの作成（左側に2つのグラフを配置）
        self.graph_frame = ttk.Frame(self.main_frame)
        self.graph_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 1つ目のグラフのフレーム
        self.figure1 = plt.Figure(figsize=(6, 6), dpi=80)
        self.ax1 = self.figure1.add_subplot(111)
        self.canvas1 = FigureCanvasTkAgg(self.figure1, self.graph_frame)
        self.canvas1.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 2つ目のグラフのフレーム
        self.figure2 = plt.Figure(figsize=(6, 6), dpi=80)
        self.ax2 = self.figure2.add_subplot(111)
        self.canvas2 = FigureCanvasTkAgg(self.figure2, self.graph_frame)
        self.canvas2.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        # グラフのレイアウトを統一するために調整
        self.figure1.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
        self.figure2.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

        # スライダ用フレームの作成（右側に配置）
        self.slider_frame = ttk.Frame(self.main_frame)
        self.slider_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # 左足と右足のスライダーを作成
        self.sliders = {}
        for leg in ['left', 'right']:
            leg_frame = ttk.LabelFrame(self.slider_frame, text=f"{leg.capitalize()} Leg")
            leg_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
            self.sliders[leg] = {}
            for angle in ['theta1', 'theta2', 'thetaF']:
                self.sliders[leg][angle] = self.create_slider_with_label(
                    f"{leg.capitalize()} {angle}", 
                    self.initial_angles[leg][angle], 
                    leg_frame
                )

        # グラフの初期設定
        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(-600, 600)
            ax.set_ylim(-1000, 200)
            ax.set_aspect('equal', adjustable='box')
            ax.set_anchor('C')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.grid(True)

        self.ax1.set_title('Extended Kinematics Visualization')
        self.ax2.set_title('Transformed Kinematics Visualization')

        self.ek_left = self.initialize_kinematics()
        self.ek_right = self.initialize_kinematics()
        self.update_plot()

    def create_slider_with_label(self, label_text, initial_value, parent_frame):
        frame = ttk.Frame(parent_frame)
        frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        label = ttk.Label(frame, text=label_text)
        label.pack(side=tk.LEFT)

        value_label = ttk.Label(frame, text=str)
        value_label.pack(side=tk.RIGHT)

        slider = ttk.Scale(frame, from_=-180, to=180, orient=tk.HORIZONTAL, 
                           command=lambda v: self.update_slider_value(v, value_label),
                           length=200)
        slider.set(initial_value)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        return slider

    def update_slider_value(self, value, value_label):
        value_label.config(text=str(int(float(value))))
        self.update_plot()

    def initialize_kinematics(self):
        Yb, l, b, m, e, f = 100, 200, 200, 400, 200, 150
        ek = ExtendedKinematics(Yb, l, b, m, e, f)
        return ek

    def update_plot(self, event=None):
        try:
            for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
                theta1 = self.sliders[leg]['theta1'].get()
                theta2 = self.sliders[leg]['theta2'].get()
                thetaF = self.sliders[leg]['thetaF'].get()
                ek.set_angles(theta1, theta2, thetaF)
                ek.compute_forward_kinematics()

            self.plot_extended_kinematics(self.ax1, self.canvas1)
            self.plot_transformed_kinematics(self.ax2, self.canvas2)

        except Exception as e:
            print(f"エラーが発生しました: {e}")

    def plot_line(self, ax, point1, point2, color, label):
        """2点間の線をプロットするヘルパー関数"""
        if point1 is not None and point2 is not None:
            ax.plot([point1[0], point2[0]], [point1[1], point2[1]], f'{color}-o', label=label)

    def plot_extended_kinematics(self, ax, canvas):
        ax.clear()

        for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
            result = ek.calculate()
            B1, M1, X, M2, B2, E, F = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["F"]

            # 線をプロット
            self.plot_line(ax, B1, M1, 'r', f'{leg} B1-M1')
            self.plot_line(ax, M1, X, 'b', f'{leg} M1-X')
            self.plot_line(ax, X, M2, 'g', f'{leg} X-M2')
            self.plot_line(ax, M2, B2, 'y', f'{leg} M2-B2')
            self.plot_line(ax, X, E, 'm', f'{leg} X-E')
            self.plot_line(ax, E, F, 'c', f'{leg} E-F')

            # ポイントをプロット
            for point, color, label in zip([B1, M1, X, M2, B2, E, F],
                                           ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan'],
                                           ['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F']):
                if point is not None:
                    ax.scatter(*point, color=color, zorder=5)
                    ax.text(point[0], point[1], f'{leg} {label}', fontsize=8, ha='right')

        ax.set_xlim(-600, 600)
        ax.set_ylim(-1000, 200)
        ax.set_anchor('C')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Extended Kinematics Visualization')
        ax.grid(True)
        ax.legend()

        self.figure1.tight_layout()
        canvas.draw()

    def plot_transformed_kinematics(self, ax, canvas):
        ax.clear()

        for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
            points = ek.format_result()
            angle_flower = angle_between_vectors((0, 0), (1, 0), points['E'], points['F'])
            transformer = Transformation2D(origin=points['E'], angle=-angle_flower)

            transformed_points = {key: transformer.transform(value) for key, value in points.items()}

            # 線をプロット
            self.plot_line(ax, transformed_points['B1'], transformed_points['M1'], 'r', f'{leg} B1-M1')
            self.plot_line(ax, transformed_points['M1'], transformed_points['X'], 'b', f'{leg} M1-X')
            self.plot_line(ax, transformed_points['X'], transformed_points['M2'], 'g', f'{leg} X-M2')
            self.plot_line(ax, transformed_points['M2'], transformed_points['B2'], 'y', f'{leg} M2-B2')
            self.plot_line(ax, transformed_points['X'], transformed_points['E'], 'm', f'{leg} X-E')
            self.plot_line(ax, transformed_points['E'], transformed_points['F'], 'c', f'{leg} E-F')

            # ポイントをプロット
            for point, color in zip(['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F'],
                                    ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan']):
                if point in transformed_points:
                    ax.scatter(*transformed_points[point], color=color, zorder=5)
                    ax.text(transformed_points[point][0], transformed_points[point][1], f'{leg} {point}', fontsize=8, ha='right')

        ax.set_xlim(-600, 600)
        ax.set_ylim(0, 1200)
        ax.set_anchor('C')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Transformed Kinematics Visualization')
        ax.grid(True)
        ax.legend()

        self.figure2.tight_layout()
        canvas.draw()

    def destroy(self):
        self.canvas1.get_tk_widget().destroy()
        self.canvas2.get_tk_widget().destroy()
        plt.close(self.figure1)
        plt.close(self.figure2)
        super().destroy()

if __name__ == '__main__':
    app = KinematicsApp()
    app.mainloop()
