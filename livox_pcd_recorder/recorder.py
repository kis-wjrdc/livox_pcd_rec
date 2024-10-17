import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2
import csv
import struct
from threading import Timer
from datetime import datetime

class LivoxCSVRecorder(Node):
    def __init__(self):
        super().__init__('livox_csv_recorder')

        # 点群データを購読
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',  # Livox LiDARの点群データトピック
            self.pointcloud_callback,
            10
        )

        # コマンド用トピックを購読
        self.command_subscription = self.create_subscription(
            Int32,
            '/record_command',  # コマンドを受け取るトピック
            self.command_callback,
            10
        )

        self.timer = None
        self.pointcloud_data = []
        self.recording = False
        self.get_logger().info('Livox CSV Recorder initialized.')

    def pointcloud_callback(self, msg):
        if self.recording:
            self.pointcloud_data.append(msg)

    def command_callback(self, msg):
        duration_ms = msg.data
        self.get_logger().info(f'Received command to record for {duration_ms} milliseconds.')
        self.start_recording(duration_ms / 1000.0)  # ミリ秒を秒に変換して記録開始

    def start_recording(self, duration):
        self.get_logger().info(f'Start recording for {duration} seconds.')
        self.recording = True
        self.pointcloud_data = []  # 以前のデータをクリア
        self.timer = Timer(duration, self.stop_recording)
        self.timer.start()

    def stop_recording(self):
        self.recording = False
        timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
        self.save_csv(f'PCD_{timestamp}.csv')
        self.get_logger().info(f'Recording stopped, file saved as CSV.')

    def save_csv(self, filename):
        # CSVファイルに保存
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z', 'intensity'])  # ヘッダーを書き込み
            for msg in self.pointcloud_data:
                for i in range(msg.width):
                    point = self.unpack_point(msg, i)
                    writer.writerow(point)
        self.get_logger().info(f'CSV file saved as {filename}.')

    def unpack_point(self, msg, index):
        # PointCloud2からXYZとIntensityをアンパック
        offset = index * msg.point_step
        x, y, z = struct.unpack_from('fff', msg.data, offset=offset)
        intensity = struct.unpack_from('f', msg.data, offset=offset + 12)[0]  # Intensity
        return [x, y, z, intensity]

def main(args=None):
    rclpy.init(args=args)
    node = LivoxCSVRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

