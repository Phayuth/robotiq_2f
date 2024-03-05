import subprocess
import time
import serial
from .modbus_crc import verify_modbus_rtu_crc
from .gripperio import GripperIO

s = subprocess.getstatusoutput("dmesg | grep -i FTDI | tail -n 1 | grep -oP 'ttyUSB\K\d+'")
usbid = s[1]
subprocess.getstatusoutput("sudo chmod a+rw /dev/ttyUSB" + usbid)


class Robotiq2F85Driver:

    def __init__(self, comport="/dev/ttyUSB" + usbid, baud=115200):
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
        """
        My lab gripper has a strange behaviour that I have to this sequence to activate properly.
        Instead of just run activate gripper.

        Run after the power-on 1 TIME ONLY.
        If run multiple time, the register will get cluter with incorrect bit.
        I don't know how to fix it since I don't understand any of this.
        """
        self.process_stat_cmd()
        while self.is_ready() is False:
            time.sleep(1)
            print("Stage [1/5]")
            self.deactivate_gripper()
            time.sleep(1)

            print("Stage [2/5]")
            self.activate_gripper()
            time.sleep(1)

            print("Stage [3/5]")
            self.deactivate_gripper()
            time.sleep(1)

            print("Stage [4/5]")
            self._gripper.goto_raw(pos=255)
            time.sleep(5)

            print("Stage [5/5]")
            self._gripper.goto_raw(pos=0)
            time.sleep(5)

            print("Stage [Done]")


if __name__ == "__main__":
    g = Robotiq2F85Driver()
    g.startup_routine()
    g.goto(0.085, 0.020, 40.0)
    time.sleep(2)
    g.goto(0.000, 0.020, 40.0)
    print("End File")
