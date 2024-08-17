import tkinter as tk
from gui import GUI
from kinematics.hip import Hip
from visualization import Visualization

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
        
        self.after(50, self.update_plot)

    def setup_kinematics(self):
        """
        ロボットのパラメータを設定する
        """

        B1 = (50, -100 + 200)
        B2 = (-50, -100 + 200)
        self.hip.set_leg_param(150, 200, 150, 150, B1, B2)

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

        ##----------左脚を基準に回転
        # 左脚の回転（PointE中心）
        points = self.hip.left_leg.get_original_points()
        self.hip.left_leg.calculate_rotated_points(points['E'], points['F'], (0, 0), (1, 0))

        # 左脚の回転後のポイント(B1,B2)
        rotate_points = self.hip.left_leg.get_rotated_points()
        
        # 右脚の回転（B1,B2中心）。B1,B2に向かって平行移動
        points = self.hip.right_leg.get_original_points()
        self.hip.right_leg.calculate_rotated_points( points['B2'], points['B1'], rotate_points['B2'], rotate_points['B1'])
        self.hip.right_leg.calculate_translate_points( rotate_points['B2'])
        ##----------左脚を基準に回転ここまで

        self.visualization.draw_transformed_kinematics(self.hip)
        self.gui.canvas.update()
        #self.after(50, self.update_plot)

