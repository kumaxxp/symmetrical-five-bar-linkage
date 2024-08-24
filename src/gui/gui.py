import tkinter as tk
from tkinter import ttk

class GUI:
    def __init__(self, master):
        self.master = master
        self.setup_ui()

    def setup_ui(self):
        self.setup_frames()
        self.setup_sliders()
        self.setup_canvas()
        self.setup_button()
        self.setup_text()

    def setup_frames(self):
        self.main_frame = ttk.Frame(self.master)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        self.left_frame = ttk.Frame(self.main_frame, width=400)
        self.left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        self.left_frame.pack_propagate(False)  # サイズを固定

        self.right_frame = ttk.Frame(self.main_frame)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

    def setup_sliders(self):
        self.sliders = {'left': {}, 'right': {}}
        self.angle_labels = {'left': {}, 'right': {}}
        for leg in ['left', 'right']:
            leg_frame = ttk.LabelFrame(self.left_frame, text=f"{leg.capitalize()} Leg")
            leg_frame.pack(fill=tk.X, padx=5, pady=5)
            for angle in ['theta1', 'theta2', 'thetaF']:
                slider = ttk.Scale(leg_frame, from_=-180, to=180, orient=tk.HORIZONTAL, 
                                   command=lambda x, l=leg, a=angle: self.on_slider_change(l, a))
                slider.pack(fill=tk.X, padx=5, pady=5)
                self.sliders[leg][angle] = slider

                label_frame = ttk.Frame(leg_frame)
                label_frame.pack(fill=tk.X, padx=5)
                ttk.Label(label_frame, text=f"{angle}:").pack(side=tk.LEFT)
                value_label = ttk.Label(label_frame, text="0.0")
                value_label.pack(side=tk.RIGHT)
                self.angle_labels[leg][angle] = value_label

    def setup_canvas(self):
        self.canvas = tk.Canvas(self.right_frame, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def on_slider_change(self, leg, angle):
        value = self.sliders[leg][angle].get()
        self.angle_labels[leg][angle].config(text=f"{value:.1f}")
        self.master.update_plot()
#        self.display_length_info()

    def setup_button(self):
        self.range_check_button = ttk.Button(self.left_frame, text="Check Range", command=self.on_range_check)
        self.range_check_button.pack(fill=tk.X, padx=5, pady=10)

    def setup_text(self):
        # 長さ情報を表示するためのテキストウィジェットを追加
        self.length_info_text = tk.Text(self.left_frame, height=10, width=50)
        self.length_info_text.pack(side=tk.BOTTOM, padx=10, pady=10)    

    def on_range_check(self):
        # kinematics_app.pyの範囲確認メソッドを呼び出す
        self.master.check_range(1)

    def set(self, leg, angle, value):
        self.sliders[leg][angle].set(value)

    def get_slider_value(self, leg, angle):
        return self.sliders[leg][angle].get()

    def update_length_info(self, length_info):
            self.length_info_text.delete('1.0', tk.END)  # テキストをクリア
            for leg in ['left', 'right']:
                leg_info = length_info[leg]
                self.length_info_text.insert(tk.END, f"{leg.capitalize()} leg:\n")
                for key, value in leg_info.items():
                    self.length_info_text.insert(tk.END, f"{key}: {value:.2f}\n")
                self.length_info_text.insert(tk.END, "\n")
