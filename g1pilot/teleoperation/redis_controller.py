#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Similar to arm_controller.py
# Try installing redis and redis-server in image
# pip install redis
# sudo apt update
# sudo apt-get install redis-server
# Then run redis in background
# redis-server --daemonize yes

"""
Redis-based teleoperation controller for G1 Robot.

Reads controller_data from Redis (key-value GET) and publishes Joy messages
to /g1pilot/joy_manual, compatible with joy_mux and loco_client.

Redis:
    Key: controller_data
    Value: JSON with LeftController, RightController (axis, axis_click,
           index_trig, grip, key_one, key_two), optional timestamp.

Velocity mapping (from controller axes):
    Left stick  -> xy movement (vx, vy)
    Right stick -> yaw rotation
"""

import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

try:
    import redis
except ImportError:
    redis = None


class RedisController(Node):
    def __init__(self):
        super().__init__("redis_controller")

        self.declare_parameter("redis_host", "localhost")
        self.declare_parameter("redis_port", 6379)
        self.declare_parameter("redis_db", 0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("xy_scale", 2.0)
        self.declare_parameter("yaw_scale", 3.0)
        # If True: button 8 (movement enable) = 1 whenever we have Redis data, so stick alone drives robot.
        # If False: button 8 = 1 only when axis_click (stick click) is pressed (hold-to-move).
        self.declare_parameter("movement_always_enabled", True)

        host = self.get_parameter("redis_host").value
        port = int(self.get_parameter("redis_port").value)
        db = int(self.get_parameter("redis_db").value)
        self.rate = float(self.get_parameter("publish_rate").value)
        self.xy_scale = float(self.get_parameter("xy_scale").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.movement_always_enabled = bool(self.get_parameter("movement_always_enabled").value)

        self.redis_client = None
        if redis is not None:
            try:
                self.redis_client = redis.Redis(host=host, port=port, db=db)
                self.redis_client.ping()
            except Exception as e:
                self.get_logger().warn(f"Redis not available ({e}). Node will run but Joy will be zeroed.")
        else:
            self.get_logger().warn("redis package not installed. pip install redis")

        # [vx, vy, vyaw] in m/s and rad/s
        self.velocity_commands = [0.0, 0.0, 0.0]
        self.move_enabled = False
        self.arms_enabled = False
        self._lock = threading.Lock()

        # Same publishers as keyboard for compatibility with joy_mux
        self.joy_pub = self.create_publisher(Joy, "/g1pilot/joy_manual", 10)
        self.balance_pub = self.create_publisher(Bool, "/g1pilot/start_balancing", 10)
        self.emergency_pub = self.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
        self.arms_enabled_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", 10)
        self.arms_home_pub = self.create_publisher(Bool, "/g1pilot/arms/home", 10)

        self.create_timer(1.0 / self.rate, self.timer_callback)
        self._log_count = 0
        self.get_logger().info(
            "Redis controller: reading controller_data from Redis, publishing to /g1pilot/joy_manual"
        )

    def _update_velocity_commands(self, controller_data):
        """Update velocity commands from controller axes."""
        left_axis = controller_data.get("LeftController", {}).get("axis", [0.0, 0.0])
        right_axis = controller_data.get("RightController", {}).get("axis", [0.0, 0.0])

        if len(left_axis) >= 2 and len(right_axis) >= 2:
            self.velocity_commands[0] = left_axis[1] * self.xy_scale
            self.velocity_commands[1] = -left_axis[0] * self.xy_scale
            self.velocity_commands[2] = -right_axis[0] * self.yaw_scale
        else:
            self.velocity_commands[0] = 0.0
            self.velocity_commands[1] = 0.0
            self.velocity_commands[2] = 0.0

    def _read_controller_data(self):
        """GET controller_data from Redis and return parsed dict or None."""
        if self.redis_client is None:
            return None
        try:
            raw = self.redis_client.get("controller_data")
            if raw is None:
                return None
            return json.loads(raw)
        except Exception:
            return None

    def _buttons_from_controller_data(self, data):
        """
        Map controller_data buttons to Joy button indices used by loco_client.
        Indices: 0=arms, 1=home, 3=gripper, 5=emergency, 6=balance, 8=movement enable.
        """
        left = data.get("LeftController", {})
        right = data.get("RightController", {})

        # Movement enable (button 8): e.g. left stick click or grip
        axis_click_left = left.get("axis_click", 0) if isinstance(left.get("axis_click"), (int, float)) else 0
        axis_click_right = right.get("axis_click", 0) if isinstance(right.get("axis_click"), (int, float)) else 0
        move_enabled = bool(axis_click_left or axis_click_right)

        # key_one / key_two: map to balance (6) and arms (0)
        key_one_left = 1 if left.get("key_one") else 0
        key_one_right = 1 if right.get("key_one") else 0
        key_two_left = 1 if left.get("key_two") else 0
        key_two_right = 1 if right.get("key_two") else 0
        balance_btn = key_one_left or key_one_right
        arms_btn = key_two_left or key_two_right

        # Home: e.g. both grips or one grip
        grip_left = 1 if left.get("grip") else 0
        grip_right = 1 if right.get("grip") else 0
        home_btn = 1 if (grip_left and grip_right) else 0

        # Emergency (5): index trigger
        index_left = 1 if left.get("index_trig") else 0
        index_right = 1 if right.get("index_trig") else 0
        emergency_btn = 1 if (index_left or index_right) else 0

        return {
            "move_enabled": move_enabled,
            "arms": arms_btn,
            "home": home_btn,
            "emergency": emergency_btn,
            "balance": balance_btn,
        }

    def timer_callback(self):
        """Read Redis, update velocity and buttons, publish Joy."""
        controller_data = self._read_controller_data()

        with self._lock:
            if controller_data is None:
                self.velocity_commands[0] = 0.0
                self.velocity_commands[1] = 0.0
                self.velocity_commands[2] = 0.0
                move_enabled = False
                arms_btn = 0
                home_btn = 0
                emergency_btn = 0
                balance_btn = 0
            else:
                self._update_velocity_commands(controller_data)
                bt = self._buttons_from_controller_data(controller_data)
                move_enabled = bt["move_enabled"]
                arms_btn = bt["arms"]
                home_btn = bt["home"]
                emergency_btn = bt["emergency"]
                balance_btn = bt["balance"]

        # loco_client only moves when buttons[8]==1. Use axis_click or always-on when we have data.
        if self.movement_always_enabled and controller_data is not None:
            move_enabled = True

        # Build Joy message (same format as keyboard / loco_client expectation)
        vx, vy, vyaw = self.velocity_commands[0], self.velocity_commands[1], self.velocity_commands[2]
        # Normalize to [-1, 1] for axes, then invert to match loco_client
        ax0 = -vy / self.xy_scale if self.xy_scale != 0 else 0.0
        ax1 = -vx / self.xy_scale if self.xy_scale != 0 else 0.0
        ax2 = -vyaw / self.yaw_scale if self.yaw_scale != 0 else 0.0
        ax0 = max(-1.0, min(1.0, ax0))
        ax1 = max(-1.0, min(1.0, ax1))
        ax2 = max(-1.0, min(1.0, ax2))

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [
            float(ax0),
            float(ax1),
            float(ax2),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        msg.buttons = [
            arms_btn,
            home_btn,
            0,
            0,
            0,
            emergency_btn,
            balance_btn,
            0,
            1 if move_enabled else 0,
            0,
            0,
            0,
        ]
        self.joy_pub.publish(msg)

        # Throttled log every ~1 s so you see Redis data status (ROS 2 shows this)
        self._log_count += 1
        if self._log_count >= int(self.rate):
            self._log_count = 0
            data_status = "yes" if controller_data is not None else "no"
            self.get_logger().info(
                f"Joy pub: redis_data={data_status} vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f} move_btn={move_enabled}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RedisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
