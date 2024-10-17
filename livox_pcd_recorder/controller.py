import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')

        # コマンドをパブリッシュするトピック
        self.publisher_ = self.create_publisher(Int32, '/record_command', 10)

        # Tkinterを使ったGUIのセットアップ
        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Livox CSV Recorder Controller")

        # ラベルとテキストボックス
        self.label = tk.Label(self.root, text="収録時間をミリ秒で入力:")
        self.label.pack()

        self.entry = tk.Entry(self.root)
        self.entry.pack()

        # ボタン
        self.button = tk.Button(self.root, text="送信", command=self.publish_command)
        self.button.pack()

        self.root.mainloop()

    def publish_command(self):
        # 収録時間の送信
        duration_ms = self.entry.get()
        if duration_ms.isdigit():
            command_msg = Int32()
            command_msg.data = int(duration_ms)
            self.publisher_.publish(command_msg)
            self.get_logger().info(f'Sent recording command: {duration_ms} milliseconds')
        else:
            self.get_logger().error("無効な入力です。数値を入力してください。")

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

