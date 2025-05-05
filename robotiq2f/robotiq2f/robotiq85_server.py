import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robotiq2f_interfaces.srv import Robotiq2FCmd, Robotiq2FInfo
from .robotiq85_driver import Robotiq2F85Driver


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


class Robotiq2FServer(Node):

    def __init__(self):
        super().__init__("robotiq_griper_server")
        # parameters
        self.declare_parameter("comport", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("use_fake_hardware", False)
        self.comport = self.get_parameter("comport").value
        self.baud = self.get_parameter("baud").value
        self.use_fake_hardware = self.get_parameter("use_fake_hardware").value

        # setup
        self.get_logger().info(bcolors.OKGREEN + "Setting Up Connection" + bcolors.ENDC)
        if not self.use_fake_hardware:
            self.gripper = Robotiq2F85Driver(self.comport, self.baud)
            self.gripper.startup_routine()

        if not self.use_fake_hardware:
            server_callback = self.server_callback
            info_callback = self.info_callback
        else:
            server_callback = self.server_fake_callback
            info_callback = self.info_fake_callback

        self.cmdSrv = self.create_service(Robotiq2FCmd, "/gripper_command", server_callback)
        self.getInfoSrv = self.create_service(Robotiq2FInfo, "/gripper_info", info_callback)

        self.timer = self.create_timer(0.01, self.gripper_state_handle)
        self.gjsPub = self.create_publisher(JointState, "/joint_states", 10)

        self.get_logger().info(f"Gripper Width : [0, 0.085] (m)")
        self.get_logger().info(f"Gripper Velo  : [0.013, 0.1] (m/s)")
        self.get_logger().info(f"Gripper Force : [5.0, 220.0] (N)")

        self._fake_req_pos = 0.0

    def close_connection(self):
        if not self.use_fake_hardware:
            self.gripper.shutdown()
        self.get_logger().info(bcolors.OKGREEN + "Gripper connection is disconnected" + bcolors.ENDC)

    def server_callback(self, request, response):
        self.get_logger().info(f"Request Gripper Width : [{request.pos}] (m)")
        self.get_logger().info(f"Request Gripper Velo  : [{request.vel}] (m/s)")
        self.get_logger().info(f"Request Gripper Force : [{request.force}] (N)")
        self.gripper.goto(pos=request.pos, vel=request.vel, force=request.force)
        response.success = True
        response.message = "Request SUCCESSFUL"
        return response

    def server_fake_callback(self, request, response):
        self.get_logger().info(f"Request Gripper Width : [{request.pos}] (m)")
        self.get_logger().info(f"Request Gripper Velo  : [{request.vel}] (m/s)")
        self.get_logger().info(f"Request Gripper Force : [{request.force}] (N)")
        self._fake_req_pos = request.pos
        response.success = True
        response.message = "Request SUCCESSFUL"
        return response

    def info_callback(self, request, response):
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

    def info_fake_callback(self, request, response):
        response.is_ready = True
        response.is_reset = True
        response.is_moving = False
        response.is_stopped = True
        response.object_detected = False
        response.get_fault_status = 0
        response.get_pos = self._fake_req_pos
        response.get_req_pos = self._fake_req_pos
        response.get_current = 0.0
        return response

    def map_val(self, x, inMin, inMax, outMin, outMax):
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

    def tcpz_from_pos(self, pos, tipdownOffset=0.0):
        z = self.map_val(pos, 0.085, 0.0, 0.1493, 0.1628)
        return z - np.clip(tipdownOffset, 0.0, 0.038)

    def joint_from_pos(self, pos):
        return np.clip(0.8 - ((0.8 / 0.085) * pos), 0.0, 0.8)

    def gripper_state_handle(self):
        time = self.get_clock().now().to_msg()

        if not self.use_fake_hardware:
            pos = self.gripper.get_pos()
        else:
            pos = self._fake_req_pos

        js = JointState()
        js.header.frame_id = ""
        js.header.stamp = time
        js.name = ["robotiq_85_left_knuckle_joint"]
        js.position = [self.joint_from_pos(pos)]
        self.gjsPub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    try:
        serviceNode = Robotiq2FServer()
        rclpy.spin(serviceNode)
    finally:
        serviceNode.close_connection()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
