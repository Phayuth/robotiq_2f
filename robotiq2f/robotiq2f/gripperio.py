import array
from .modbus_crc import compute_modbus_rtu_crc, verify_modbus_rtu_crc


class SignalValueConverter:
    """
    Convert Signal value [0-255] to Real Physical Value based on each Gripper Specific Parameter.
    - lerp = linear interpolation
    - rlerp = reverse linear interpolation

    Return value is clamped to avoid over physical limit using min and max.

    """

    def conv_signal_to_real_lerp(signal, realmin, realmax):
        real = realmin + signal * ((realmax - realmin) / 255)
        return max(realmin, min(real, realmax))

    def conv_real_to_signal_lerp(real, realmin, realmax):
        signal = (real - realmin) * (255 / (realmax - realmin))
        return int(max(0, min(signal, 255)))

    def conv_signal_to_real_rlerp(signal, realmin, realmax):
        real = realmin + (signal - 255) * ((realmax - realmin) / (-255))
        return max(realmin, min(real, realmax))

    def conv_real_to_signal_rlerp(real, realmin, realmax):
        signal = (real - realmax) * (255 / (realmin - realmax))
        return int(max(0, min(signal, 255)))


class GripperIO:

    def __init__(self, device, posMin, posMax, velMin, velMax, forceMin, forceMax):
        self.ACTION_REQ_IDX = 7
        self.POS_INDEX = 10
        self.SPEED_INDEX = 11
        self.FORCE_INDEX = 12

        self.device = device + 9
        self.rPR = 0  # goto position request 0 fully open, 255 fully close
        self.rSP = 255  # speed request 0 minimum 255 maximum
        self.rFR = 150  # force request 0 minimum 255 maximum
        self.rARD = 1  # auto release direction 0==closing, 1==open
        self.rATR = 0  # auto release 0==normal, 1==emergency
        self.rGTO = 0  # move gripper to req pos with defined config, 0==stop, 1==go to req pos
        self.rACT = 0  # activation, 0 == deact, 1==act
        self.gSTA = 0  # get gripper status, 0 reset(fault) 1 act in progress, 2 not in used, 3 act completed
        self.gACT = 0  # get activation status 0==gripper reset 1==gripper activation
        self.gGTO = 0  # get goto 0 stopped(performing act or auto release) 1 goto pose req
        self.gOBJ = 0  # object detection, ignore if gGTO==0. 0 move to reqpos no obj det. 1 stop while opening obj det, 2 stop while closing obj det, 3 arrived at req pos
        self.gFLT = 0  # 0 not fault (solide blue), 5 delayed must activate prior, 7 mul
        self.gPO = 0  # get current pos
        self.gPR = 0  # get pos req
        self.gCU = 0  # get current
        self.act_cmd = [0] * 0x19
        self.act_cmd[:7] = [self.device, 0x10, 0x03, 0xE8, 0x00, 0x08, 0x10]
        self.act_cmd_bytes = b""
        self._update_cmd()

        # Physical Parameter
        self.posMin = posMin
        self.posMax = posMax
        self.velMin = velMin
        self.velMax = velMax
        self.forceMin = forceMin
        self.forceMax = forceMax

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
        posr = SignalValueConverter.conv_real_to_signal_rlerp(pos, self.posMin, self.posMax)
        velr = SignalValueConverter.conv_real_to_signal_lerp(vel, self.velMin, self.velMax)
        forcer = SignalValueConverter.conv_real_to_signal_lerp(force, self.forceMin, self.forceMax)
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
        return SignalValueConverter.conv_signal_to_real_rlerp(po, self.posMin, self.posMax)

    def get_req_pos(self):
        pr = float(self.gPR)
        return SignalValueConverter.conv_signal_to_real_rlerp(pr, self.posMin, self.posMax)

    def get_current(self):
        return self.gCU * 0.1

    def _update_action_req(self):
        self._act_req = self.rACT | (self.rGTO << 3) | (self.rATR << 4) | (self.rARD << 5)

    def _update_cmd(self):
        self._update_action_req()
        self.act_cmd = self.act_cmd[: len(self.act_cmd) - 2]
        self.act_cmd[self.ACTION_REQ_IDX] = self._act_req & 0x39
        self.act_cmd[self.POS_INDEX] = self.rPR & 0xFF
        self.act_cmd[self.SPEED_INDEX] = self.rSP & 0xFF
        self.act_cmd[self.FORCE_INDEX] = self.rFR & 0xFF
        compute_modbus_rtu_crc(self.act_cmd)
        self.act_cmd_bytes = array.array("B", self.act_cmd).tobytes()

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
