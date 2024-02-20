import can
import rclpy
from std_msgs.msg import Float32, Int16

class CanStatus:
    def __init__(self):
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.node = rclpy.create_node('can_data_publisher')

        self.speed_publisher = self.node.create_publisher(Float32, 'speed_status', 10)
        self.steering_publisher = self.node.create_publisher(Float32, 'steering_status', 10)
        self.battery_publisher = self.node.create_publisher(Int16, 'battery_value', 10)

    def process_can_message(self, message):
        if message.arbitration_id == 0x304 and len(message.data) == 8:
            # Extracting the 4th byte with bit length 16 (bits 32-47) for steering angle
            msg1 = message.data[5]
            msg2 = message.data[4]
            hex_string = hex(msg1)[2:] + hex(msg2)[2:]
            steering_value = int(hex_string, 16)

            steering_value = int(steering_value) * 0.1 - 35

            speed_bits = (message.data[1] << 8) | message.data[0]

            # Convert to speed to km/h with a resolution of 0.1
            speed_decimal = speed_bits / 10.0
            speed_decimal = speed_decimal - 80  # offset value from the datasheet

            # Publishing data
            speed_msg = Float32()
            speed_msg.data = float(round(speed_decimal))
            self.speed_publisher.publish(speed_msg)

            steering_msg = Float32()
            steering_msg.data = float(round(steering_value))
            self.steering_publisher.publish(steering_msg)

        if message.arbitration_id == 0x303 and len(message.data) == 8:  # Getting Battery Value
            battery_value = Int16()
            battery_value.data = message.data[6]
            self.battery_publisher.publish(battery_value)

    def run(self):
        while rclpy.ok():
            message = self.bus.recv(0.2)  # Timeout in seconds.

            if message is not None:
                self.process_can_message(message)

        rclpy.spin(self.node)
        rclpy.shutdown()

def main():
    rclpy.init()
    can_status = CanStatus()
    can_status.run()

if __name__ == '__main__':
    main()
