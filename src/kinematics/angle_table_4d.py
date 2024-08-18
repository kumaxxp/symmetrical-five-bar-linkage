import numpy as np

class AngleTable4D:
    def __init__(self, min_angle=-180.0, max_angle=180.0, step=1.0, data_callback=None):
        """
        コンストラクタで角度範囲、刻み幅、データコールバックを設定する。
        
        :param min_angle: 角度範囲の最小値 (デフォルト -180.0)
        :param max_angle: 角度範囲の最大値 (デフォルト 180.0)
        :param step: 角度の刻み幅 (デフォルト 1.0)
        :param data_callback: 各角度で生成するデータを返すコールバック関数
        """
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.step = step
        self.data_callback = data_callback if data_callback else self.default_data_callback

        # 角度の範囲を生成
        self.angle_range = np.arange(self.min_angle, self.max_angle + self.step, self.step)
        self.dim_size = len(self.angle_range)
        
        # 角度をインデックスに変換する辞書を作成
        self.angle_to_index = {angle: i for i, angle in enumerate(self.angle_range)}
        
        # 4次元テーブルの初期化
        self.table_4d = np.empty((self.dim_size, self.dim_size, self.dim_size, self.dim_size), dtype=object)

    def default_data_callback(self, angle1, angle2, angle3, angle4):
        """
        デフォルトのデータコールバック関数。
        各角度に対応する辞書型データを生成。
        """
        return {
            "angle_1": angle1,
            "angle_2": angle2,
            "angle_3": angle3,
            "angle_4": angle4,
            "sum": angle1 + angle2 + angle3 + angle4,
            "product": angle1 * angle2 * angle3 * angle4
        }

    def generate_table(self):
        """
        テーブルを生成する。
        コールバック関数を使ってデータを生成し、テーブルに格納する。
        エラーが発生した場合は空の辞書を格納する。
        """
        for i in range(self.dim_size):
            for j in range(self.dim_size):
                for k in range(self.dim_size):
                    for l in range(self.dim_size):
                        # 各次元に対応する角度を取得
                        angle_i = self.angle_range[i]
                        angle_j = self.angle_range[j]
                        angle_k = self.angle_range[k]
                        angle_l = self.angle_range[l]
                        
                        try:
                            # コールバック関数を使用してデータを生成
                            result_dict = self.data_callback(angle_i, angle_j, angle_k, angle_l)
                        except Exception as e:
                            print(f"Error occurred for angles: {angle_i}, {angle_j}, {angle_k}, {angle_l}. Error: {e}")
                            result_dict = {}  # エラーが発生した場合は空の用
                        
                        self.table_4d[i, j, k, l] = result_dict


    def get_data_by_angle(self, angle1, angle2, angle3, angle4):
        """
        指定された角度に対応するテーブルのデータを取得する。
        
        :param angle1: 第1次元の角度
        :param angle2: 第2次元の角度
        :param angle3: 第3次元の角度
        :param angle4: 第4次元の角度
        :return: 指定された角度に対応する辞書型データ
        """
        # 角度をインデックスに変換
        index1 = self.angle_to_index.get(angle1)
        index2 = self.angle_to_index.get(angle2)
        index3 = self.angle_to_index.get(angle3)
        index4 = self.angle_to_index.get(angle4)
        
        # インデックスが有効であるか確認
        if index1 is not None and index2 is not None and index3 is not None and index4 is not None:
            return self.table_4d[index1, index2, index3, index4]
        else:
            print(angle1, angle2, angle3, angle4)
            raise ValueError("指定された角度が範囲外です。")

    def __repr__(self):
        return f"AngleTable4D(min_angle={self.min_angle}, max_angle={self.max_angle}, step={self.step})"

# 上位クラスの例
class AngleTableManager:
    def __init__(self, step=1.0):
        """
        AngleTableManagerは、AngleTable4Dの管理を行い、テーブル生成を呼び出す。
        
        :param step: 角度の刻み幅 (デフォルト 1.0)
        """
        self.angle_table = AngleTable4D(step=step, data_callback=self.custom_data_callback)

    def custom_data_callback(self, angle1, angle2, angle3, angle4):
        """
        カスタムのデータコールバック関数。
        任意のロジックを使用して、テーブルに格納するデータを生成する。
        """
        # 任意のカスタム計算ロジック
        return {
            "angle_1": angle1,
            "angle_2": angle2,
            "angle_3": angle3,
            "angle_4": angle4,
            "sum_of_squares": angle1**2 + angle2**2 + angle3**2 + angle4**2,
            "max_angle": max(angle1, angle2, angle3, angle4)
        }

    def create_table(self):
        """
        テーブルを生成するメソッド。
        """
        self.angle_table.generate_table()

    def get_data(self, angle1, angle2, angle3, angle4):
        """
        指定された角度に対応するデータを取得するメソッド。
        """
        return self.angle_table.get_data_by_angle(angle1, angle2, angle3, angle4)

# 使用例
if __name__ == "__main__":
    # 上位クラスのインスタンスを作成
    manager = AngleTableManager(step=10.0)

    # テーブルを生成
    manager.create_table()

    # 角度で直接データを取得
    angle1 = 20.0
    angle2 = 20.0
    angle3 = 20.0
    angle4 = 20.0

    result = manager.get_data(angle1, angle2, angle3, angle4)
    print(f"角度 ({angle1}, {angle2}, {angle3}, {angle4}) に対応するデータ: {result}")
