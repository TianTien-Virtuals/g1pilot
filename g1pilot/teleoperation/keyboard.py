#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard Teleoperation for G1 Robot

Controls:
    Movement (hold SPACE + keys):
        W / ↑  : Forward
        S / ↓  : Backward
        A / ←  : Strafe Left
        D / →  : Strafe Right
        Q      : Rotate Left
        E      : Rotate Right

    Actions:
        SPACE  : Enable movement (hold)
        B      : Start balancing
        X      : Toggle arm control
        H      : Home arms
        ESC    : Emergency stop

    Speed:
        1-5    : Set speed level (1=slow, 5=fast)
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False



class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publishers
        self.joy_pub = self.create_publisher(Joy, '/g1pilot/joy_manual', 10)
        self.start_pub = self.create_publisher(Bool, '/g1pilot/start', 10)
        self.balance_pub = self.create_publisher(Bool, '/g1pilot/start_balancing', 10)
        self.emergency_pub = self.create_publisher(Bool, '/g1pilot/emergency_stop', 10)
        self.arms_enabled_pub = self.create_publisher(Bool, '/g1pilot/arms/enabled', 10)
        self.arms_home_pub = self.create_publisher(Bool, '/g1pilot/arms/home', 10)

        # Parameters
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_vyaw', 0.5)

        self.rate = self.get_parameter('publish_rate').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_vyaw = self.get_parameter('max_vyaw').value

        # State
        self.keys_pressed = set()
        self.speed_level = 3  # 1-5, default medium
        self.move_enabled = False  # SPACE held
        self.arms_enabled = False
        self.lock = threading.Lock()

        # Timer to publish Joy messages
        self.create_timer(1.0 / self.rate, self.publish_joy)

        # Print controls
        self.print_controls()

    def print_controls(self):
        controls = """
╔════════════════════════════════════════════════════════════════╗
║              KEYBOARD TELEOPERATION - G1 ROBOT                 ║
╠════════════════════════════════════════════════════════════════╣
║  MOVEMENT (hold SPACE + direction keys):                       ║
║    W / ↑     : Forward         S / ↓     : Backward            ║
║    A / ←     : Strafe Left     D / →     : Strafe Right        ║
║    Q         : Rotate Left     E         : Rotate Right        ║
║                                                                ║
║  ACTIONS:                                                      ║
║    SPACE     : Enable movement (hold)                          ║
║    B         : Start balancing                                 ║
║    X         : Toggle arm control                              ║
║    H         : Home arms                                       ║
║    ESC       : Emergency stop                                  ║
║                                                                ║
║  SPEED:                                                        ║
║    1-5       : Set speed level (current: 3)                    ║
╚════════════════════════════════════════════════════════════════╝
"""
        self.get_logger().info(controls)

    def get_speed_multiplier(self):
        """Returns speed multiplier based on current level (0.2 to 1.0)"""
        return 0.2 + (self.speed_level - 1) * 0.2

    def on_key_press(self, key):
        with self.lock:
            try:
                if hasattr(key, 'char') and key.char:
                    k = key.char.lower()
                    self.keys_pressed.add(k)

                    # Speed levels
                    if k in '12345':
                        self.speed_level = int(k)
                        self.get_logger().info(f'Speed level: {self.speed_level}')

                    # Actions
                    if k == 'b':
                        self.get_logger().info('Starting balancing...')
                        self.balance_pub.publish(Bool(data=True))
                    elif k == 'x':
                        self.arms_enabled = not self.arms_enabled
                        state = "ENABLED" if self.arms_enabled else "DISABLED"
                        self.get_logger().info(f'Arm control: {state}')
                        self.arms_enabled_pub.publish(Bool(data=self.arms_enabled))
                    elif k == 'h':
                        self.get_logger().info('Homing arms...')
                        self.arms_home_pub.publish(Bool(data=True))

                elif key == keyboard.Key.space:
                    if not self.move_enabled:
                        self.move_enabled = True
                        self.get_logger().info('Movement ENABLED (hold SPACE)')
                elif key == keyboard.Key.escape:
                    self.get_logger().warn('EMERGENCY STOP!')
                    self.emergency_pub.publish(Bool(data=True))
                elif key == keyboard.Key.up:
                    self.keys_pressed.add('up')
                elif key == keyboard.Key.down:
                    self.keys_pressed.add('down')
                elif key == keyboard.Key.left:
                    self.keys_pressed.add('left')
                elif key == keyboard.Key.right:
                    self.keys_pressed.add('right')

            except AttributeError:
                pass

    def on_key_release(self, key):
        with self.lock:
            try:
                if hasattr(key, 'char') and key.char:
                    k = key.char.lower()
                    self.keys_pressed.discard(k)
                elif key == keyboard.Key.space:
                    self.move_enabled = False
                    self.get_logger().info('Movement DISABLED')
                elif key == keyboard.Key.up:
                    self.keys_pressed.discard('up')
                elif key == keyboard.Key.down:
                    self.keys_pressed.discard('down')
                elif key == keyboard.Key.left:
                    self.keys_pressed.discard('left')
                elif key == keyboard.Key.right:
                    self.keys_pressed.discard('right')
            except AttributeError:
                pass

    def compute_velocity(self):
        """Compute vx, vy, vyaw from pressed keys"""
        vx = 0.0
        vy = 0.0
        vyaw = 0.0

        with self.lock:
            keys = self.keys_pressed.copy()
            enabled = self.move_enabled

        if not enabled:
            return 0.0, 0.0, 0.0

        speed = self.get_speed_multiplier()

        # Forward/Backward
        if 'w' in keys or 'up' in keys:
            vx = self.max_vx * speed
        if 's' in keys or 'down' in keys:
            vx = -self.max_vx * speed

        # Strafe Left/Right
        if 'a' in keys or 'left' in keys:
            vy = self.max_vy * speed
        if 'd' in keys or 'right' in keys:
            vy = -self.max_vy * speed

        # Rotate
        if 'q' in keys:
            vyaw = self.max_vyaw * speed
        if 'e' in keys:
            vyaw = -self.max_vyaw * speed

        return vx, vy, vyaw

    def publish_joy(self):
        """Publish Joy message mimicking joystick format expected by loco_client"""
        vx, vy, vyaw = self.compute_velocity()

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Match the joystick format expected by loco_client.py:
        # axes[0] = vy (inverted in loco_client)
        # axes[1] = vx (inverted in loco_client)
        # axes[2] = yaw (inverted in loco_client)
        # Note: loco_client multiplies by -0.5, so we need to account for that
        # It expects raw joystick values -1 to 1

        # Normalize to joystick range and invert (loco_client inverts them)
        ax0 = -vy / self.max_vy if self.max_vy != 0 else 0.0  # vy
        ax1 = -vx / self.max_vx if self.max_vx != 0 else 0.0  # vx
        ax2 = -vyaw / self.max_vyaw if self.max_vyaw != 0 else 0.0  # yaw

        # Create axes array (8 axes like typical gamepad)
        msg.axes = [
            float(ax0),   # 0: vy
            float(ax1),   # 1: vx
            float(ax2),   # 2: yaw
            0.0,          # 3: unused
            0.0,          # 4: unused (L2/R2 trigger for gripper)
            0.0,          # 5: unused
            0.0,          # 6: unused
            0.0,          # 7: D-pad
        ]

        # Create buttons array (12 buttons like typical gamepad)
        # Button 8 must be 1 to enable movement in loco_client
        msg.buttons = [
            0,  # 0: X (toggle arms)
            0,  # 1: O (home arms)
            0,  # 2: Square
            0,  # 3: Triangle (gripper)
            0,  # 4: L1
            0,  # 5: R1 (emergency)
            0,  # 6: L2 (balance)
            0,  # 7: R2
            1 if self.move_enabled else 0,  # 8: Share/Select - enables movement!
            0,  # 9: Options/Start
            0,  # 10: L3
            0,  # 11: R3
        ]

        self.joy_pub.publish(msg)



def main(args=None):
    if not PYNPUT_AVAILABLE:
        print("ERROR: pynput library not found!")
        print("Install it with: pip install pynput")
        return

    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Start keyboard listener in background
    listener = keyboard.Listener(
        on_press=node.on_key_press,
        on_release=node.on_key_release
    )
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
