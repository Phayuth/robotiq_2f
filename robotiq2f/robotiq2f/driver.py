import array
import subprocess
import time
import numpy as np
import serial
from .modbus_crc import compute_modbus_rtu_crc, verify_modbus_rtu_crc

s = subprocess.getstatusoutput("dmesg | grep -i FTDI | tail -n 1 | grep -oP 'ttyUSB\K\d+'")
usbid = s[1]
subprocess.getstatusoutput("sudo chmod a+rw /dev/ttyUSB" + usbid)

ACTION_REQ_IDX = 7
POS_INDEX = 10
SPEED_INDEX = 11
FORCE_INDEX = 12


class GripperIO:

    def __init__(self, device):
        self.device = device + 9
        self.rPR = 0   # goto position request 0 fully open, 255 fully close
        self.rSP = 255 # speed request 0 minimum 255 maximum
        self.rFR = 150 # force request 0 minimum 255 maximum
        self.rARD = 1  # auto release direction 0==closing, 1==open
        self.rATR = 0  # auto release 0==normal, 1==emergency
        self.rGTO = 0  # move gripper to req pos with defined config, 0==stop, 1==go to req pos
        self.rACT = 0  # activation, 0 == deact, 1==act
        self.gSTA = 0  # get gripper status, 0 reset(fault) 1 act in progress, 2 not in used, 3 act completed
        self.gACT = 0  # get activation status 0==gripper reset 1==gripper activation
        self.gGTO = 0  # get goto 0 stopped(performing act or auto release) 1 goto pose req
        self.gOBJ = 0  # object detection, ignore if gGTO==0. 0 move to reqpos no obj det. 1 stop while opening obj det, 2 stop while closing obj det, 3 arrived at req pos
        self.gFLT = 0  # 0 not fault (solide blue), 5 delayed must activate prior, 7 mul
        self.gPO = 0   # get current pos
        self.gPR = 0   # get pos req
        self.gCU = 0   # get current
        self.act_cmd = [0] * 0x19
        self.act_cmd[:7] = [self.device, 0x10, 0x03, 0xE8, 0x00, 0x08, 0x10]
        self.act_cmd_bytes = b""
        self._update_cmd()

        # Physic Parameter
        self.posMin = 0.0
        self.posMax = 0.085
        self.velMin = 0.013
        self.velMax = 0.1
        self.forceMin = 5.0
        self.forceMax = 220.0

        # Description from Manual:
        # self.device = SlaveID
        # 0x03        = Function Code 03, Read Holding Registers
        # 0x07D0      = Address of the first requested register
        # 0x0008      = Number of registers requested
        # Note that there isn't a Cyclic Redundance Check (adds 0xC5CE to the end)
        self.stat_cmd = [self.device, 0x03, 0x07, 0xD0, 0x00, 0x08]
        compute_modbus_rtu_crc(self.stat_cmd)
        self.stat_cmd_bytes = array.array("B", self.stat_cmd).tobytes()

    def activate_gripper(self):
        self.rACT = 1
        self.rPR = 0
        self.rSP = 255
        self.rFR = 150
        self._update_cmd()

    def deactivate_gripper(self):
        self.rACT = 0
        self._update_cmd()

    def activate_emergency_release(self, open_gripper=True):
        self.rATR = 1
        self.rARD = 1

        if open_gripper:
            self.rARD = 0
        self._update_cmd()

    def deactivate_emergency_release(self):
        self.rATR = 0
        self._update_cmd()

    def goto(self, pos, vel, force):
        posr = int(np.clip((3.0 - 230.0) / self.posMax * pos + 230.0, 0, 255))
        velr = int(np.clip(255.0 / (self.velMax - self.velMin) * vel - self.velMin, 0, 255))
        forcer = int(np.clip(255.0 / (self.forceMax - self.forceMin) * force - self.forceMin, 0, 255))
        self.goto_raw(posr, velr, forcer)

    def goto_raw(self, pos, vel, force):  # control in raw data 0 -> 255
        self.rACT = 1
        self.rGTO = 1
        self.rPR = pos
        self.rSP = vel
        self.rFR = force
        self._update_cmd()

    def stop(self):
        self.rACT = 1
        self.rGTO = 0
        self._update_cmd()

    def parse_rsp(self, rsp):
        if verify_modbus_rtu_crc(rsp):
            self.gACT = rsp[3] & 0x1
            self.gGTO = (rsp[3] & 0x8) >> 3
            self.gSTA = (rsp[3] & 0x30) >> 4
            self.gOBJ = (rsp[3] & 0xC0) >> 6
            self.gFLT = rsp[5] & 0x0F
            self.gPR = rsp[6] & 0xFF
            self.gPO = rsp[7] & 0xFF
            self.gCU = rsp[8] & 0xFF
            return True
        return False

    def is_ready(self):
        return self.gSTA == 3 and self.gACT == 1

    def is_reset(self):
        return self.gSTA == 0 or self.gACT == 0

    def is_moving(self):
        return self.gGTO == 1 and self.gOBJ == 0

    def is_stopped(self):
        return self.gOBJ != 0

    def object_detected(self):
        return self.gOBJ == 1 or self.gOBJ == 2

    def get_status(self):
        return self.gSTA

    def get_fault_status(self):
        return self.gFLT

    def get_pos(self):
        po = float(self.gPO)
        return np.clip(self.posMax / (3.0 - 230.0) * (po - 230.0), self.posMin, self.posMax)

    def get_req_pos(self):
        pr = float(self.gPR)
        return np.clip(self.posMax / (3.0 - 230.0) * (pr - 230.0), self.posMin, self.posMax)

    def get_current(self):
        return self.gCU * 0.1

    def _update_action_req(self):
        self._act_req = self.rACT | (self.rGTO << 3) | (self.rATR << 4) | (self.rARD << 5)

    def _update_cmd(self):
        self._update_action_req()
        self.act_cmd = self.act_cmd[: len(self.act_cmd) - 2]
        self.act_cmd[ACTION_REQ_IDX] = self._act_req & 0x39
        self.act_cmd[POS_INDEX] = self.rPR & 0xFF
        self.act_cmd[SPEED_INDEX] = self.rSP & 0xFF
        self.act_cmd[FORCE_INDEX] = self.rFR & 0xFF
        compute_modbus_rtu_crc(self.act_cmd)
        self.act_cmd_bytes = array.array("B", self.act_cmd).tobytes()


class Robotiq2F85Driver:

    def __init__(self, comport="/dev/ttyUSB" + usbid, baud=115200):
        try:
            self.ser = serial.Serial(comport, baud, timeout=0.2)
        except:
            self.init_success = False
            return

        self._gripper = GripperIO(0)
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
    g._gripper.goto_raw(pos=255)
    time.sleep(2)
    print("End File")
