#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time


class MockMcu(Node):
    def __init__(self):
        super().__init__('mock_mcu')
        self.get_logger().info("Mock MCU Node started.")

        self.declare_parameter('port', '/tmp/ttyV1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.get_logger().info(f"Connecting to virtual serial port: {port} at {baudrate}")

        # State
        self.temp_cf = 25.0
        self.temp_resin = 25.0
        self.target_cf = 25.0
        self.target_resin = 25.0
        self.fan_ok_cf = 1
        self.fan_ok_resin = 1
        self.tool = 0
        self.err = 0

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
        except Exception as e:
            self.get_logger().error(f"Failed to open port {port}: {e}")
            raise

        self.running = True
        self.thread = threading.Thread(target=self.mcu_loop)
        self.thread.start()

    def mcu_loop(self):
        last_stat_time = time.time()
        stat_interval = 0.1  # 10Hz

        buffer = ""

        while self.running and rclpy.ok():
            # Read from serial
            try:
                data = self.ser.read(1024)
                if data:
                    buffer += data.decode('ascii', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.handle_line(line)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

            # Update temperatures (simulate heating/cooling)
            now = time.time()
            dt = now - last_stat_time
            if dt >= stat_interval:
                # Simulate 5 degrees per second heating
                rate = 5.0 * dt
                if self.temp_cf < self.target_cf:
                    self.temp_cf = min(self.target_cf, self.temp_cf + rate)
                elif self.temp_cf > self.target_cf:
                    self.temp_cf = max(self.target_cf, self.temp_cf - rate)

                if self.temp_resin < self.target_resin:
                    self.temp_resin = min(self.target_resin, self.temp_resin + rate)
                elif self.temp_resin > self.target_resin:
                    self.temp_resin = max(self.target_resin, self.temp_resin - rate)

                # Send STAT
                stat_msg = (
                    f"STAT temp_cf={self.temp_cf:.1f} target_cf={self.target_cf:.1f} "
                    f"temp_resin={self.temp_resin:.1f} "
                    f"target_resin={self.target_resin:.1f} "
                    f"fan_ok_cf={self.fan_ok_cf} "
                    f"fan_ok_resin={self.fan_ok_resin} "
                    f"tool={self.tool} err={self.err}\n"
                )
                try:
                    self.ser.write(stat_msg.encode('ascii'))
                except Exception as e:
                    self.get_logger().error(f"Serial write error: {e}")

                last_stat_time = now

            time.sleep(0.005)  # Small sleep to avoid 100% CPU

    def handle_line(self, line):
        self.get_logger().debug(f"MCU received: {line}")
        parts = line.split()
        if not parts:
            return

        cmd = parts[0]
        if cmd == "EV":
            # EV <trigger_seq> <event_type> <arg>
            if len(parts) >= 3:
                ev_type = parts[2]
                arg = parts[3] if len(parts) > 3 else "0"

                if ev_type == "heat_cf":
                    self.target_cf = float(arg)
                elif ev_type == "heat_resin":
                    self.target_resin = float(arg)
                elif ev_type == "tool_change_cf" or ev_type == "tool_change_resin":
                    self.tool = int(arg)
                elif ev_type == "fan_cf":
                    self.fan_ok_cf = 1 if arg == "1" else 0
                elif ev_type == "fan_resin":
                    self.fan_ok_resin = 1 if arg == "1" else 0

        elif cmd == "E":
            # Extrusion command, we just acknowledge it implicitly by staying alive
            pass

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
