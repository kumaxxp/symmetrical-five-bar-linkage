import tkinter as tk
from gui import GUI
from kinematics.hip import Hip
from visualization import Visualization
from kinematics.angle_table_4d import AngleTable4D

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
        
        # 角度テーブルを用意して全部の角度の組み合わせを計算
        # 後でGUIの呼び出し時に計算する
        self.angle_table = AngleTable4D(step=10, data_callback=self.custom_data_callback)
        self.angle_table.generate_table()

        self.after(50, self.update_plot)

    def custom_data_callback(self, angle1, angle2, angle3, angle4):
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
        self.hip.set_leg_param(150, 200, 150, 150, B1, B2)
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

        self.visualization.draw_transformed_kinematics(self.hip)
        self.gui.canvas.update()
