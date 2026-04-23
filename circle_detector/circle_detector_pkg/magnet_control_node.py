from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

try:
    import gpiod
except ImportError:
    gpiod = None


class MagnetControlNode(Node):
    """Subscribes to /magnet_command (Bool). True -> energize (low), False -> release (high)."""

    def __init__(self):
        super().__init__('magnet_control')

        self.declare_parameter('gpio_chip', 'gpiochip1')
        self.declare_parameter('gpio_line', 4)
        self.declare_parameter('default_on', False)

        chip_name = self.get_parameter('gpio_chip').value
        line_offset = int(self.get_parameter('gpio_line').value)
        default_on = bool(self.get_parameter('default_on').value)

        self._line = None
        if gpiod is None:
            self.get_logger().error(
                'python3-gpiod not installed; magnet_control will only log state.'
            )
        else:
            try:
                chip = gpiod.Chip(chip_name)
                self._line = chip.get_line(line_offset)
                self._line.request(consumer='magnet_control', type=gpiod.LINE_REQ_DIR_OUT)
            except Exception as e:
                self.get_logger().error(f'Failed to acquire gpio line {chip_name}/{line_offset}: {e}')
                self._line = None

        self._set_magnet(default_on)

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub = self.create_subscription(Bool, '/magnet_command', self._on_command, latched)

        self.get_logger().info(
            f'MagnetControlNode started chip={chip_name} line={line_offset} default_on={default_on}'
        )

    def _set_magnet(self, energize: bool) -> None:
        # Low level energizes the magnet, high level releases it.
        value = 0 if energize else 1
        if self._line is not None:
            try:
                self._line.set_value(value)
            except Exception as e:
                self.get_logger().error(f'Failed to set gpio value: {e}')
                return
        self.get_logger().info(f'Magnet -> {"ON (low)" if energize else "OFF (high)"}')

    def _on_command(self, msg: Bool) -> None:
        self._set_magnet(bool(msg.data))

    def destroy_node(self) -> None:
        # Release magnet on shutdown so payload drops cleanly.
        self._set_magnet(False)
        if self._line is not None:
            try:
                self._line.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MagnetControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
