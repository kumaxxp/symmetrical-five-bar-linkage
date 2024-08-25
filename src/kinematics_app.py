import tkinter as tk
from gui import GUI
from kinematics.hip import Hip
from visualization import Visualization
from kinematics.angle_table_4d import AngleTable4D

import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Kinematics Visualization')
        self.geometry('1600x1000')
        self.resizable(False, True)

        self.gui = GUI(self)
        self.hip = Hip()
        self.visualization = Visualization(self.gui.canvas)

        self.setup_kinematics()
        self.set_initial_angles()  # GUIのスライダに初期値を設定
        self.initialize_kinematics()  # 初期値を使用して運動学を初期化
        
        """
        # 角度テーブルを用意して全部の角度の組み合わせを計算
        # 後でGUIの呼び出し時に計算する
        self.angle_table = AngleTable4D(step=10, 
            data_callback_before=self.custom_data_callback_before, 
            data_callback_after=self.custom_data_callback_after)
        
        self.angle_table.generate_table()
        # テーブルをCSVとJSONファイルに保存
        self.angle_table.save_to_csv('angle_table.csv')
        self.angle_table.save_to_json('angle_table.json')
        """

        self.after(50, self.update_plot)

    def custom_data_callback_before(self, angle1, angle2):
        angleF = self.gui.get_slider_value('left', 'thetaF')

        self.hip.set_leg_angles('left', angle1, angle2, angleF)

        try:
            # スライダコントロールの値に沿って、リンクの基本形状を計算する
            self.hip.left_leg.compute_forward_kinematics()
            original_dict = self.hip.left_leg.get_original_points()

            # 計算結果の辞書型を設定する
            result_dict = {
                "angle_1": angle1,
                "angle_2": angle2,
                "angle_3": 0,
                "angle_4": 0,
                "angle_F": angleF,
                "left_points": original_dict,
                "right_points": original_dict,
                "center_com": (0,0)
            }

        except Exception as e:
            print(f"Error occurred for angles: {angle1}, {angle2}. Error: {e}")
            result_dict = None

        return result_dict

    def custom_data_callback_after(self, angle1, angle2, angle3, angle4):
        """
        カスタムのデータコールバック関数。
        任意のロジックを使用して、テーブルに格納するデータを生成する。
        """

        angleF = self.gui.get_slider_value('left', 'thetaF')

        self.hip.set_leg_angles('left', angle1, angle2, angleF)
        self.hip.set_leg_angles('right', angle3, angle4, angleF)

        try:
            # スライダコントロールの値に沿って、リンクの基本形状を計算する
            self.hip.compute_forward_kinematics()

            # 左脚を基準に回転
            self.hip.align_legs_to_ground()

            # 重心を計算する
            self.hip.calculate_total_center_of_mass()

            rotated_dict = self.hip.get_rotated_points()

            # 計算結果の辞書型を設定する
            result_dict = {
                "angle_1": angle1,
                "angle_2": angle2,
                "angle_3": angle3,
                "angle_4": angle4,
                "angle_F": angleF,
                "left_points": rotated_dict['left'],
                "right_points": rotated_dict['right'],
                "center_com": self.hip.center_com
            }

        except Exception as e:
            print(f"Error occurred for angles: {angle1}, {angle2}, {angle3}, {angle4}. Error: {e}")
            result_dict = {}

        return result_dict

    def setup_kinematics(self):
        """
        ロボットのパラメータを設定する
        """

        B1 = (50, -100 + 200)
        B2 = (-50, -100 + 200)
        W1 = (70, -100 + 200)
        W2 = (-70, -100 + 200)
        self.hip.set_leg_param(150, 200, 150, 150, B1, B2, W1, W2, 20)
        self.hip.set_weights({'B1': 1, 'B2': 1, 'E': 1})

        self.initial_angles = {
            'left': {'theta1': -50, 'theta2': -120, 'thetaF': -60},
            'right': {'theta1': -50, 'theta2': -120, 'thetaF': -60}
        }

    def set_initial_angles(self):
        for leg in ['left', 'right']:
            for angle, value in self.initial_angles[leg].items():
                self.gui.set(leg, angle, value)

    def initialize_kinematics(self):
        for leg in ['left', 'right']:
            initial_angles = {
                'theta1': self.gui.get_slider_value(leg, 'theta1'),
                'theta2': self.gui.get_slider_value(leg, 'theta2'),
                'thetaF': self.gui.get_slider_value(leg, 'thetaF')
            }
            self.hip.set_leg_angles(leg, **initial_angles)
        self.hip.compute_forward_kinematics()

    def update_plot(self):
        for leg in ['left', 'right']:
            current_angles = {
                'theta1': self.gui.get_slider_value(leg, 'theta1'),
                'theta2': self.gui.get_slider_value(leg, 'theta2'),
                'thetaF': self.gui.get_slider_value(leg, 'thetaF')
            }
            self.hip.set_leg_angles(leg, **current_angles)

        # スライダコントロールの値に沿って、リンクの基本形状を計算する
        self.hip.compute_forward_kinematics()

        # 左脚を基準に回転
        self.hip.align_legs_to_ground()

        # 重心を計算する
        self.hip.calculate_total_center_of_mass()

        # 長さ情報を取得
        length_info = self.hip.display_length_info()

        # GUIに長さ情報を反映
        self.gui.update_length_info(length_info)        

        self.visualization.draw_transformed_kinematics(self.hip)
        self.gui.canvas.update()

    def check_range(self, step=1):
        """
        リンクの移動範囲を確認する
        step: 角度の刻み幅（デフォルトは1度）
        """
        angleF = 60

        # プログレスバーを作成
        progress_window = tk.Toplevel(self)
        progress_window.title('Calculating...')
        progress_var = tk.DoubleVar()
        total_steps = (360 // step) * (360 // step)
        progress_bar = ttk.Progressbar(progress_window, variable=progress_var, maximum=total_steps)
        progress_bar.pack(pady=10, padx=10, fill=tk.X)

        # 計算を別スレッドで開始
        thread = threading.Thread(target=self._calculate_range, args=(progress_window, progress_var, angleF, step))
        thread.start()

    def _calculate_range(self, progress_window, progress_var, angleF, step):
        # angle1,angle2の組み合わせで有効/無効を記載する2次元配列を初期化
        size = 360 // step
        valid_combinations = np.zeros((size, size), dtype=bool)

        count = 0
        for i, angle1 in enumerate(range(-180, 180, step)):
            for j, angle2 in enumerate(range(-180, 180, step)):
                try:
                    self.hip.set_leg_angles('left', angle1, angle2, angleF)
                    self.hip.left_leg.compute_forward_kinematics()
                    valid_combinations[i, j] = True
                except Exception as e:
                    pass

                count += 1
                progress_var.set(count)
                progress_window.update()  # GUIを更新

        progress_window.destroy()
        self.show_results(valid_combinations, step)

    def show_results(self, valid_combinations, step):
        result_window = tk.Toplevel(self)
        result_window.title('Valid Combinations')

        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(valid_combinations, cmap='RdYlGn', interpolation='nearest', origin='lower')
        ax.set_xlabel('Angle 2')
        ax.set_ylabel('Angle 1')
        ax.set_title(f'Valid Angle Combinations (Step: {step}°)')

        # Set tick labels
        size = 360 // step
        tick_step = max(1, size // 6)  # 6つ以下のティックになるように調整
        ax.set_xticks(np.arange(0, size + 1, tick_step))
        ax.set_yticks(np.arange(0, size + 1, tick_step))
        ax.set_xticklabels(np.arange(-180, 181, tick_step * step))
        ax.set_yticklabels(np.arange(-180, 181, tick_step * step))

        plt.colorbar(im)

        canvas = FigureCanvasTkAgg(fig, master=result_window)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(fill=tk.BOTH, expand=True)

        valid_count = np.sum(valid_combinations)
        total_count = valid_combinations.size
        percentage = (valid_count / total_count) * 100

        result_label = tk.Label(result_window, text=f"Valid combinations: {valid_count}/{total_count} ({percentage:.2f}%)")
        result_label.pack()

