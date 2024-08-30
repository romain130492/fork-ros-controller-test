import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from time import sleep

print('re')

class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.get_logger().info('Node created')

    def send_commands(self):
        commands = Float64MultiArray()

        # Initial command to ensure gripper is reset
        commands.data = [0.0]
        self.publisher.publish(commands)
        sleep(1)

        # Command to set the gripper position to 0.38
        commands.data = [0.38]
        self.publisher.publish(commands)
        sleep(1)

        # Command to set the gripper position to 0.19
        commands.data = [0.19]
        self.publisher.publish(commands)
        sleep(1)

        # Command to reset the gripper position to 0
        commands.data = [0.0]
        self.publisher.publish(commands)
        sleep(1)

def main(args=None):
    rclpy.init(args=args)
    gripper_test_node = GripperTestNode()
    try:
        gripper_test_node.send_commands()
    finally:
        gripper_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
