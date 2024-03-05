import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from robotiq2f_interfaces.srv import Robotiq2FCmd, Robotiq2FInfo
from .robotiqhande_driver import RobotiqHandEDriver


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class RobotiqHandEServer(Node):

    def __init__(self):
        super().__init__("robotiq_griper_server")
        self.get_logger().info(bcolors.OKGREEN + "Setting Up Connection" + bcolors.ENDC)
        self.gripper = RobotiqHandEDriver()
        self.gripper.startup_routine()

        self.getInfoSrv = self.create_service(Robotiq2FInfo, "/gripper_info", self.get_info_cb)
        self.cmdSrv = self.create_service(Robotiq2FCmd, "/gripper_command", self.cmd_cb)

        self.tfbroadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.gripper_state_handle)
        self.gjsPub = self.create_publisher(JointState, "/gripper/joint_states", 10)

        self.get_logger().info(f"Grip Width : [0, 0.050] (m)")
        self.get_logger().info(f"Grip Velo  : [0.020, 0.150] (m/s)")
        self.get_logger().info(f"Grip Force : [60.0, 130.0] (N)")

    def close_connection(self):
        self.gripper.shutdown()
        self.get_logger().info(bcolors.OKGREEN + "Gripper connection is disconnected" + bcolors.ENDC)

    def get_info_cb(self, request, response):
        response.is_ready = self.gripper.is_ready()
        response.is_reset = self.gripper.is_reset()
        response.is_moving = self.gripper.is_moving()
        response.is_stopped = self.gripper.is_stopped()
        response.object_detected = self.gripper.object_detected()
        response.get_fault_status = self.gripper.get_fault_status()
        response.get_pos = self.gripper.get_pos()
        response.get_req_pos = self.gripper.get_req_pos()
        response.get_current = self.gripper.get_current()
        return response

    def cmd_cb(self, request, response):
        self.get_logger().info(f"Request Gripper Width : [{request.pos}] (m)")
        self.get_logger().info(f"Request Gripper Velo  : [{request.vel}] (m/s)")
        self.get_logger().info(f"Request Gripper Force : [{request.force}] (N)")
        self.gripper.goto(pos=request.pos, vel=request.vel, force=request.force)
        response.success = True
        response.message = "Request SUCCESSFUL"
        return response

    def joint_from_pos(self, pos):
        return np.clip(0.025 - (pos/2.0), 0., 0.025)

    def gripper_state_handle(self):
        time = self.get_clock().now().to_msg()
        pos = self.gripper.get_pos()

        js = JointState()
        js.header.frame_id = ""
        js.header.stamp = time
        js.name = ["hande_joint_finger"]
        js.position = [self.joint_from_pos(pos)]
        self.gjsPub.publish(js)

        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = "gripper_adapter_link"
        t.child_frame_id = "tcp"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.146
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tfbroadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    try:
        serviceNode = RobotiqHandEServer()
        rclpy.spin(serviceNode)
    finally:
        serviceNode.close_connection()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
