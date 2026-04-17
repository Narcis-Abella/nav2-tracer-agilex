#!/usr/bin/env python3
"""
E-Stop node — monitors GPIO pin on Jetson Orin NX.
Button pressed  → cancels active navigation goal + publishes zero velocity.
Button released → allows new navigation goals (no auto-resume).

Requires: Jetson.GPIO (sudo pip3 install Jetson.GPIO)
GPIO pin: configurable via 'gpio_pin' parameter (default: 18)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class EStopNode(Node):
    def __init__(self):
        super().__init__('estop_node')
        self.declare_parameter('gpio_pin', 18)
        self._pin = self.get_parameter('gpio_pin').value
        self._estop_active = False

        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                self._pin,
                GPIO.BOTH,
                callback=self._gpio_callback,
                bouncetime=50
            )
            self.get_logger().info(f'EStop monitoring GPIO pin {self._pin}')
        else:
            self.get_logger().warn('Jetson.GPIO not available — E-stop disabled. Install: pip3 install Jetson.GPIO')

        # Publish zero vel at 10Hz while estop active
        self._timer = self.create_timer(0.1, self._estop_tick)

    def _gpio_callback(self, channel):
        if not GPIO_AVAILABLE:
            return
        pressed = GPIO.input(channel) == GPIO.LOW
        if pressed and not self._estop_active:
            self._estop_active = True
            self.get_logger().warn('E-STOP PRESSED — cancelling navigation')
            self._cancel_navigation()
        elif not pressed and self._estop_active:
            self._estop_active = False
            self.get_logger().info('E-stop released — ready for new goals')

    def _cancel_navigation(self):
        if self._nav_client.server_is_ready():
            self.get_logger().info('Sending cancel to NavigateToPose action server')
            self._nav_client._cancel_all_goals()
        zero = Twist()
        self._vel_pub.publish(zero)

    def _estop_tick(self):
        if self._estop_active:
            self._vel_pub.publish(Twist())

    def __del__(self):
        if GPIO_AVAILABLE:
            GPIO.cleanup()


def main():
    rclpy.init()
    node = EStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
