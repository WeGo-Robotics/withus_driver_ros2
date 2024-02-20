import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import time

class CanSubscriber(Node):
    def __init__(self):
        self.bus= can.interface.Bus(channel='can0', bustype='socketcan')
        
        super().__init__('can_subscriber')
        self.declare_parameter('break', 0)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Received Twist message: Linear: %f, Angular: %f' %
                               (msg.linear.x, msg.angular.z))
        self.send_command(msg.linear.x,msg.angular.z)
        
    
        # Process the received Twist message (update CAN data if needed)

    def send_command(self,linear_vel,angular_vel):
        gear = None
        indicator = None
        if angular_vel < -15:
            indicator= 0xF2
        elif angular_vel > 15:
            indicator = 0xF1
        else:
            indicator = 0x0
        if linear_vel < 0: #BACKWARD Reverse Gear
            gear = 0x3
            linear_vel = abs(linear_vel)
            #print (linear_vel)
        elif linear_vel == 0 and angular_vel == 0: # Neutral Gear
            gear = 0x2
        else:                                      # Forward Driving Gear 
            gear = 0x1
        linear_velocity = linear_vel
        linear_velocity = int(linear_velocity/0.1)
        linear_v2 = hex((linear_velocity & 0xFF00) >> 8)
        linear_v1 = hex(linear_velocity & 0x00FF)
        linear_v2 = int(linear_v2[2:4], 16)
        linear_v1 = int(linear_v1[2:4], 16)

        angular_velocity = angular_vel
        angular_velocity = int((angular_velocity + 30) / 0.1)
        angular_v2 = hex((angular_velocity & 0xFF00) >> 8)
        angular_v1 = hex(angular_velocity & 0x00FF)
        angular_v2 = int(angular_v2[2:4], 16)
        angular_v1 = int(angular_v1[2:4], 16)

        # Create CAN messages
        enable_msg = can.Message(
            arbitration_id=0x501, data=[0xF1, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False
        )
        steering_msg = can.Message(
            arbitration_id=0x502, data=[0xF1, 0, 0, 0, angular_v1, angular_v2, 0, 0], is_extended_id=False
        )
        speed_message = can.Message(
            arbitration_id=0x504, data=[0xF1, 0x00, 0x01, gear, 0x00, 0x00, linear_v1, linear_v2], is_extended_id=False
        )

        break_command = can.Message(
            arbitration_id=0x503, data=[0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False
        )
        indicator_cmd = can.Message(
            arbitration_id=0x506, data=[indicator, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False
        )
        # Create a CAN bus
        break_value = self.get_parameter('break').value
        if break_value == 1:
            self.break_call()
        else:
            self.bus.send(enable_msg)
            self.bus.send(steering_msg)
            self.bus.send(speed_message)
            self.bus.send(break_command) # To disable break if exists
            self.bus.send(indicator_cmd)
    def break_call(self):
        break_command_10 = can.Message(
            arbitration_id=0x503, data=[0xF1, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False
        ) ##Break Value is 10%
        indicator_command_10 = can.Message(
            arbitration_id=0x506, data=[0x0, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False
        ) ##Break Value is 10%
        self.bus.send(break_command_10)
        self.bus.send(indicator_command_10)
        # time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    can_subscriber = CanSubscriber()
    try:
        # send_one()
        rclpy.spin(can_subscriber)
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Shutting down...")
        can_subscriber.break_call() ##Calling Break to Stop

    finally:
        can_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()