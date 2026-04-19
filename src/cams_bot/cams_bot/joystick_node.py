#!/usr/bin/env python3
"""
Input Driver Node
Reads Xbox controller or keyboard input and publishes to /joy topic

Launch:
  ros2 run joystick_control joystick_node --ros-args -p input_mode:=xbox
  ros2 run joystick_control joystick_node --ros-args -p input_mode:=keyboard
"""

import os
import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame


# ── Keyboard bindings ─────────────────────────────────────────────────────────
# Each key maps to (channel, value)
KEY_BINDINGS = {
    'w': ('vx',    1.0),   # forward
    's': ('vx',   -1.0),   # backward
    'a': ('vy',    1.0),   # strafe left
    'd': ('vy',   -1.0),   # strafe right
    'q': ('omega', 1.0),   # rotate CCW
    'e': ('omega',-1.0),   # rotate CW
    ' ': ('stop',  0.0),   # stop
}

MOVE_KEYS = {k for k, (ch, _) in KEY_BINDINGS.items() if ch != 'stop'}


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.declare_parameter('input_mode',   'xbox')
        self.declare_parameter('device_id',    0)
        self.declare_parameter('publish_rate', 100.0)

        self.input_mode  = self.get_parameter('input_mode').value
        device_id        = self.get_parameter('device_id').value
        publish_rate     = self.get_parameter('publish_rate').value

        if self.input_mode == 'xbox':
            self._init_xbox(device_id)
        elif self.input_mode == 'keyboard':
            self._init_keyboard()
        else:
            raise ValueError(f"Unknown input_mode '{self.input_mode}'. Use 'xbox' or 'keyboard'.")

        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.timer     = self.create_timer(1.0 / publish_rate, self.publish_joy)

        self.get_logger().info(f'Input node ready! mode={self.input_mode} @ {publish_rate} Hz')

    # ── Xbox init ─────────────────────────────────────────────────────────────
    def _init_xbox(self, device_id):
        os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
        os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No controllers detected!')
            raise RuntimeError('No joystick found')

        self.joystick = pygame.joystick.Joystick(device_id)
        self.joystick.init()
        self.get_logger().info(f'Controller: {self.joystick.get_name()}')
        self.get_logger().info(f'  Axes: {self.joystick.get_numaxes()}  Buttons: {self.joystick.get_numbuttons()}')

    # ── Keyboard init ─────────────────────────────────────────────────────────
    def _init_keyboard(self):
        self._vx    = 0.0
        self._vy    = 0.0
        self._omega = 0.0
        self._kb_lock = threading.Lock()

        # Save terminal settings and switch to raw mode
        self._fd       = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)

        # Read keypresses in a background thread
        self._kb_thread = threading.Thread(target=self._keyboard_reader, daemon=True)
        self._kb_thread.start()

        self.get_logger().info('Keyboard mode active (SSH-safe raw input):')
        self.get_logger().info('  W/S    Forward / Backward')
        self.get_logger().info('  A/D    Strafe left / right')
        self.get_logger().info('  Q/E    Rotate CCW / CW')
        self.get_logger().info('  SPACE  Stop')
        self.get_logger().info('  CTRL+C Quit')

    def _keyboard_reader(self):
        """Background thread: reads raw keypresses from stdin."""
        while rclpy.ok():
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break

            if ch == '\x03':  # CTRL+C
                rclpy.shutdown()
                break

            with self._kb_lock:
                binding = KEY_BINDINGS.get(ch)
                if binding is None:
                    # Key released or unknown — zero only movement axes
                    self._vx    = 0.0
                    self._vy    = 0.0
                    self._omega = 0.0
                elif binding[0] == 'stop':
                    self._vx = self._vy = self._omega = 0.0
                elif binding[0] == 'vx':
                    self._vx = binding[1]
                elif binding[0] == 'vy':
                    self._vy = binding[1]
                elif binding[0] == 'omega':
                    self._omega = binding[1]

    # ── Publish ───────────────────────────────────────────────────────────────
    def publish_joy(self):
        if self.input_mode == 'xbox':
            pygame.event.pump()
            axes    = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        else:
            with self._kb_lock:
                # Axis layout to match manual_control_node:
                #   axes[0] = vy   (left stick X)
                #   axes[1] = -vx  (left stick Y, negated in manual_control_node)
                #   axes[2] = unused
                #   axes[3] = omega (right stick X)
                axes    = [self._vy, -self._vx, 0.0, self._omega]
                buttons = [0] * 8

        msg                 = Joy()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joystick'
        msg.axes            = axes
        msg.buttons         = buttons
        self.publisher.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Shutting down input node')
        if self.input_mode == 'keyboard':
            # Restore terminal settings
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        else:
            pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoystickNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
