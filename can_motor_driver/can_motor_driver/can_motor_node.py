import rclpy
from rclpy.node import Node
import can
import struct

# MKS CAN command IDs (from your firmware)
MKS_CMD_POSITION_COMMAND = 0xF5
MKS_CMD_POSITION_READ    = 0x31
MKS_CMD_VELOCITY_READ    = 0x32
MKS_CMD_ENABLE_DISABLE   = 0xF3
MKS_CMD_EMERGENCY_STOP   = 0xF7
MKS_CMD_CALIBRATION      = 0xF8

# Example CAN IDs (configurable)
DEFAULT_CAN_ID = 0x01
DEFAULT_ACK_ID = 0xBE

# CRC calculation: sum of CAN_ID (lower 8 bits) + data bytes, & 0xFF
def calc_crc(can_id, data):
    crc = (can_id & 0xFF) + sum(data)
    return crc & 0xFF

class CANMotorNode(Node):
    def __init__(self):
        super().__init__('can_motor_node')
        self.declare_parameter('can_id', DEFAULT_CAN_ID)
        self.declare_parameter('ack_id', DEFAULT_ACK_ID)
        self.declare_parameter('can_interface', 'can0')
        self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        self.ack_id = self.get_parameter('ack_id').get_parameter_value().integer_value
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
        self.get_logger().info(f'CANMotorNode started on {self.can_interface} with CAN ID {self.can_id}')
        # Subscribers for commands
        self.create_subscription(
            # ...
            # Add your topics here, e.g. std_msgs/Int32 for position, Bool for enable, etc.
            # Example:
            # std_msgs.msg.Int32, '/motor_X/position_command', self.position_command_cb, 10
            # ...
            # For brevity, not implemented in this starter
            # You can add: enable/disable, position, velocity, calibration, emergency stop
            # ...
            # For now, just listen to CAN and print responses
            can_msgs.msg.Frame, '/can_rx', self.can_rx_cb, 10
        )
        self.create_timer(0.1, self.poll_can)

    def send_can_command(self, cmd_id, data):
        frame = [cmd_id] + list(data)
        # Pad to 7 bytes, CRC is 8th
        while len(frame) < 7:
            frame.append(0)
        crc = calc_crc(self.can_id, frame)
        frame.append(crc)
        msg = can.Message(arbitration_id=self.can_id, data=frame, is_extended_id=False)
        self.bus.send(msg)
        self.get_logger().info(f'Sent CAN: {frame} (cmd {cmd_id:02X})')

    def poll_can(self):
        # Non-blocking read
        msg = self.bus.recv(timeout=0.01)
        if msg:
            self.get_logger().info(f'Received CAN: ID={msg.arbitration_id:02X} Data={list(msg.data)}')
            # TODO: Parse and publish to ROS topics

    def can_rx_cb(self, msg):
        # Placeholder for incoming CAN messages if using a bridge
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CANMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
