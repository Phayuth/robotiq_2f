import time
import serial
from .modbus_crc import verify_modbus_rtu_crc
from .gripperio import GripperIO


class Robotiq2F85Driver:

    def __init__(self, comport="/dev/ttyUSB", baud=115200):
        try:
            self.ser = serial.Serial(comport, baud, timeout=0.2)
        except:
            self.init_success = False
            return

        # Robotiq 85 Physical Parameter
        self.posMin = 0.000
        self.posMax = 0.085
        self.velMin = 0.013
        self.velMax = 0.1
        self.forceMin = 5.0
        self.forceMax = 220.0

        self._gripper = GripperIO(0, self.posMin, self.posMax, self.velMin, self.velMax, self.forceMin, self.forceMax)
        self.init_success = True
        self._shutdown_driver = False

    def shutdown(self):
        self._shutdown_driver = True
        self.ser.close()

    def process_act_cmd(self):
        if self._shutdown_driver:
            return False
        try:
            self.ser.write(self._gripper.act_cmd_bytes)
            rsp = self.ser.read(8)
            rsp = [int(x) for x in rsp]
            if len(rsp) != 8:
                return False
            return verify_modbus_rtu_crc(rsp)
        except:
            return False

    def process_stat_cmd(self):
        try:
            self.ser.write(self._gripper.stat_cmd_bytes)
            rsp = self.ser.read(21)
            rsp = [int(x) for x in rsp]
            if len(rsp) != 21:
                return False
            return self._gripper.parse_rsp(rsp)
        except:
            return False

    def activate_gripper(self):
        self._gripper.activate_gripper()
        self.process_act_cmd()

    def deactivate_gripper(self):
        self._gripper.deactivate_gripper()
        self.process_act_cmd()

    def activate_emergency_release(self, open_gripper=True):
        self._gripper.activate_emergency_release(open_gripper)
        self.process_act_cmd()

    def deactivate_emergency_release(self):
        self._gripper.deactivate_emergency_release()
        self.process_act_cmd()

    def goto(self, pos, vel=0, force=0):
        self._gripper.goto(pos, vel, force)
        self.process_act_cmd()

    def stop(self):
        self._gripper.stop()
        self.process_act_cmd()

    def is_ready(self):
        self.process_stat_cmd()
        return self._gripper.is_ready()

    def is_reset(self):
        self.process_stat_cmd()
        return self._gripper.is_reset()

    def is_moving(self):
        self.process_stat_cmd()
        return self._gripper.is_moving()

    def is_stopped(self):
        self.process_stat_cmd()
        return self._gripper.is_moving()

    def object_detected(self):
        self.process_stat_cmd()
        return self._gripper.object_detected()

    def get_status(self):
        self.process_stat_cmd()
        return self._gripper.get_status()

    def get_fault_status(self):
        self.process_stat_cmd()
        return self._gripper.get_fault_status()

    def get_pos(self):
        self.process_stat_cmd()
        return self._gripper.get_pos()

    def get_req_pos(self):
        self.process_stat_cmd()
        return self._gripper.get_req_pos()

    def get_current(self):
        self.process_stat_cmd()
        return self._gripper.get_current()

    def startup_routine(self):
        self.deactivate_gripper()
        time.sleep(1)

        self.activate_gripper()
        time.sleep(1)
