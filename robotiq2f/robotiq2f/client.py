import sys
import rclpy
from rclpy.node import Node
from robotiq2f_interfaces.srv import Robotiq2FCmd, Robotiq2FInfo


class OnrobotSGClient(Node):

    def __init__(self):
        super().__init__("robotiq_gripper_client")
        self.cli = self.create_client(Robotiq2FCmd, "/gripper_command")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_request(self, pos, vel, force):
        reqMsg = Robotiq2FCmd.Request()
        reqMsg.pos = pos
        reqMsg.vel = vel
        reqMsg.force = force
        self.future = self.cli.call_async(reqMsg)
        return self.future


def main(args=None):
    rclpy.init(args=args)
    clientNode = OnrobotSGClient()
    pos = float(sys.argv[1])
    vel = float(sys.argv[2])
    force = float(sys.argv[3])
    future = clientNode.send_request(pos, vel, force)

    while rclpy.ok():
        rclpy.spin_once(clientNode)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                clientNode.get_logger().info(f"Service call failed {(e,)}")
            else:
                clientNode.get_logger().info(f"Status is : {response.message}")
            break

    clientNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
