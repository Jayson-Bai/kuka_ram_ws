#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from my_project_interfaces.msg import RsiHeartBeat


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=20.0, help="mm/s")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds")
    parser.add_argument("--rate", type=float, default=250.0, help="Hz (250Hz=4ms)")
    parser.add_argument("--topic", type=str, default="/rsi/heartbeat")
    parser.add_argument("--tool-id", type=int, default=0)
    parser.add_argument("--start-seq", type=int, default=1)
    parser.add_argument("--start-extrude", type=float, default=0.0)
    parser.add_argument("--ipoc", type=str, default="0")
    args = parser.parse_args()

    rclpy.init()
    node = Node("heartbeat_sim")
    pub = node.create_publisher(RsiHeartBeat, args.topic, 10)

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_t = t0
    seq = args.start_seq

    try:
        while rclpy.ok():
            now = time.monotonic()
            if now - t0 >= args.duration:
                break
            if now < next_t:
                time.sleep(next_t - now)
                continue

            elapsed = now - t0
            msg = RsiHeartBeat()
            msg.stamp = node.get_clock().now().to_msg()
            msg.ipoc = args.ipoc
            msg.seq_used = seq
            msg.tool_id = args.tool_id
            msg.extrude_abs = args.start_extrude + args.speed * elapsed
            pub.publish(msg)

            seq += 1
            next_t += period
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
