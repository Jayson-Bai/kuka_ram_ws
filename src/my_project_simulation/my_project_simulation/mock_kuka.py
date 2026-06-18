#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import time
import re
import threading
from std_msgs.msg import String


class MockKuka(Node):
    def __init__(self):
        super().__init__('mock_kuka')
        self.get_logger().info("Mock KUKA RSI Node started.")

        self.declare_parameter('remote_ip', '127.0.0.1')
        self.declare_parameter('remote_port', 49152)

        self.remote_ip = self.get_parameter('remote_ip').value
        self.remote_port = self.get_parameter('remote_port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to an ephemeral port to receive replies
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.002)  # 2ms timeout for fast non-blocking recv

        self.kuka_running = False
        self.create_subscription(String, '/system/command', self.on_command, 10)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0

        self.ipoc = 1

        self.running = True
        self.thread = threading.Thread(target=self.udp_loop)
        self.thread.start()

    def on_command(self, msg):
        if msg.data == "RESUME" and not self.kuka_running:
            self.kuka_running = True
            self.get_logger().info(
                "Mock KUKA RSI received start signal (RESUME). "
                "Starting UDP transmission..."
            )
        elif msg.data == "PAUSE":
            self.get_logger().info("Mock KUKA RSI received PAUSE.")
        elif msg.data == "ABORT":
            self.kuka_running = False
            self.get_logger().info("Mock KUKA RSI received ABORT.")

    def udp_loop(self):
        period = 0.004  # 4ms

        while self.running and rclpy.ok():
            if not self.kuka_running:
                time.sleep(0.1)
                continue

            start_time = time.perf_counter()

            # Send XML to rsi_node
            xml = (
                f'<Rob><RIst X="{self.x:.5f}" Y="{self.y:.5f}" '
                f'Z="{self.z:.5f}" A="{self.a:.5f}" '
                f'B="{self.b:.5f}" C="{self.c:.5f}" />'
                f'<IPOC>{self.ipoc}</IPOC></Rob>'
            )
            try:
                self.sock.sendto(xml.encode('utf-8'), (self.remote_ip, self.remote_port))
            except Exception as e:
                self.get_logger().error(f"Send error: {e}")

            # Try to receive reply
            try:
                data, _ = self.sock.recvfrom(4096)
                reply_xml = data.decode('utf-8')

                # Parse PosCorr from reply
                match_x = re.search(r'X="([^"]+)"', reply_xml)
                match_y = re.search(r'Y="([^"]+)"', reply_xml)
                match_z = re.search(r'Z="([^"]+)"', reply_xml)
                match_a = re.search(r'A="([^"]+)"', reply_xml)
                match_b = re.search(r'B="([^"]+)"', reply_xml)
                match_c = re.search(r'C="([^"]+)"', reply_xml)

                if match_x:
                    self.x = float(match_x.group(1))
                if match_y:
                    self.y = float(match_y.group(1))
                if match_z:
                    self.z = float(match_z.group(1))
                if match_a:
                    self.a = float(match_a.group(1))
                if match_b:
                    self.b = float(match_b.group(1))
                if match_c:
                    self.c = float(match_c.group(1))

            except socket.timeout:
                pass  # Expected if no reply yet
            except Exception as e:
                self.get_logger().error(f"Recv error: {e}")

            self.ipoc += 1

            # Precise sleep for 4ms cycle
            elapsed = time.perf_counter() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockKuka()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
