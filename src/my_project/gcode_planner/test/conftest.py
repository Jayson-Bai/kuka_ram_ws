import sys
import types


if "rclpy" not in sys.modules:
    rclpy_stub = types.ModuleType("rclpy")
    rclpy_stub.init = lambda *args, **kwargs: None
    rclpy_stub.spin = lambda *args, **kwargs: None
    rclpy_stub.shutdown = lambda *args, **kwargs: None
    node_stub = types.ModuleType("rclpy.node")

    class _Node:
        def __init__(self, *args, **kwargs):
            pass

    node_stub.Node = _Node
    rclpy_stub.node = node_stub
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["rclpy.node"] = node_stub
