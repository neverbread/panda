#!/usr/bin/env python3
import unittest
from panda import Panda, DLC_TO_LEN
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda
from panda.tests.safety.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase


# 4 bit checkusm used in some hyundai messages
# lives outside the can packer because we never send this msg
def checksum(msg):
  addr, t, dat, bus = msg

  chksum = 0
  if addr == 0x386:
    for i, b in enumerate(dat):
      for j in range(8):
        # exclude checksum and counter bits
        if (i != 1 or j < 6) and (i != 3 or j < 6) and (i != 5 or j < 6) and (i != 7 or j < 6):
          bit = (b >> j) & 1
        else:
          bit = 0
        chksum += bit
    chksum = (chksum ^ 9) & 0xF
    ret = bytearray(dat)
    ret[5] |= (chksum & 0x3) << 6
    ret[7] |= (chksum & 0xc) << 4
  else:
    for i, b in enumerate(dat):
      if addr in [0x260, 0x421] and i == 7:
        b &= 0x0F if addr == 0x421 else 0xF0
      elif addr == 0x394 and i == 6:
        b &= 0xF0
      elif addr == 0x394 and i == 7:
        continue
      chksum += sum(divmod(b, 16))
    chksum = (16 - chksum) % 16
    ret = bytearray(dat)
    ret[6 if addr == 0x394 else 7] |= chksum << (4 if addr == 0x421 else 0)

  return addr, t, ret, bus


class TestHyundaiSafety(HyundaiButtonBase, common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  RELAY_MALFUNCTION_ADDRS = {0: (0x340,)}  # LKAS11
  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 7
  MAX_TORQUE = 384
  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000
  DRIVER_TORQUE_ALLOWANCE = 50
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2
  MIN_VALID_STEERING_RT_INTERVAL = 810000  # a ~10% buffer, can send steer up to 110Hz

  cnt_gas = 0
  cnt_speed = 0
  cnt_brake = 0
  cnt_cruise = 0
  cnt_button = 0

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI, 0)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=0):
    values = {"CF_Clu_CruiseSwState": buttons, "CF_Clu_CruiseSwMain": main_button, "CF_Clu_AliveCnt1": self.cnt_button}
    self.__class__.cnt_button += 1
    return self.packer.make_can_msg_panda("CLU11", bus, values)

  def _user_gas_msg(self, gas):
    values = {"CF_Ems_AclAct": gas, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_panda("EMS16", 0, values, fix_checksum=checksum)

  def _user_brake_msg(self, brake):
    values = {"DriverOverride": 2 if brake else 0, "AliveCounterTCS": self.cnt_brake % 8}
    self.__class__.cnt_brake += 1
    return self.packer.make_can_msg_panda("TCS13", 0, values, fix_checksum=checksum)

  def _speed_msg(self, speed):
    # panda safety doesn't scale, so undo the scaling
    values = {"WHL_SPD_%s" % s: speed * 0.03125 for s in ["FL", "FR", "RL", "RR"]}
    values["WHL_SPD_AliveCounter_LSB"] = (self.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_panda("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": enable, "CR_VSM_Alive": self.cnt_cruise % 16}
    self.__class__.cnt_cruise += 1
    return self.packer.make_can_msg_panda("SCC12", self.SCC_BUS, values, fix_checksum=checksum)

  def _torque_driver_msg(self, torque):
    values = {"CR_Mdps_StrColTq": torque}
    return self.packer.make_can_msg_panda("MDPS12", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"CR_Lkas_StrToqReq": torque, "CF_Lkas_ActToi": steer_req}
    return self.packer.make_can_msg_panda("LKAS11", 0, values)


class TestHyundaiSafetyAltLimits(TestHyundaiSafety):
  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE = 270

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI, Panda.FLAG_HYUNDAI_ALT_LIMITS)
    self.safety.init_tests()


class TestHyundaiSafetyCameraSCC(TestHyundaiSafety):
  BUTTONS_TX_BUS = 2  # tx on 2, rx on 0
  SCC_BUS = 2  # rx on 2

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI, Panda.FLAG_HYUNDAI_CAMERA_SCC)
    self.safety.init_tests()


class TestHyundaiLegacySafety(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI_LEGACY, 0)
    self.safety.init_tests()


class TestHyundaiLegacyEngineTemperaturesSafety(TestHyundaiLegacySafety):
  TX_MSGS = TestHyundaiLegacySafety.TX_MSGS + [[0x7E0, 1], [0x7DF, 1], [0x7E1, 1]]
  TEMPERATURE_MSGS = (
    (0x7E0, bytes.fromhex("03 22 E0 01 00 00 00 00")),
    (0x7DF, bytes.fromhex("02 01 05 00 00 00 00 00")),
    (0x7E0, bytes.fromhex("30 00 0A 00 00 00 00 00")),
    (0x7E1, bytes.fromhex("03 22 01 A4 00 00 00 00")),
    (0x7E1, bytes.fromhex("30 00 0A 00 00 00 00 00")),
  )

  def setUp(self):
    super().setUp()
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI_LEGACY, Panda.FLAG_HYUNDAI_ENGINE_TEMPERATURES)
    self.safety.init_tests()

  def test_temperature_queries_with_controls_on_and_off(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for addr, dat in self.TEMPERATURE_MSGS:
        self.assertEqual(self._tx(libpanda_py.make_CANPacket(addr, 1, dat)), 1)
        self.assertEqual(self.safety.get_controls_allowed(), controls_allowed)
      for addr, dat in ((0x7E8, "10 33 62 E0 01 00 00 00"), (0x7E9, "10 31 62 01 A4 00 00 00")):
        self._rx(libpanda_py.make_CANPacket(addr, 1, bytes.fromhex(dat)))
        self.assertEqual(self.safety.get_controls_allowed(), controls_allowed)

  def test_temperature_query_payloads_exact(self):
    # Mutate every byte, including all ISO-TP fields and padding. Only the five
    # complete messages are permitted, not arbitrary reads or diagnostic writes.
    for addr, dat in self.TEMPERATURE_MSGS:
      for index in range(8):
        for value in range(256):
          if value == dat[index]:
            continue
          changed = bytearray(dat)
          changed[index] = value
          self.assertEqual(self._tx(libpanda_py.make_CANPacket(addr, 1, changed)), 0,
                           (hex(addr), index, changed.hex()))

  def test_temperature_query_bus_and_length(self):
    for addr, dat in self.TEMPERATURE_MSGS:
      for bus in range(8):
        for length in DLC_TO_LEN:
          msg = libpanda_py.make_CANPacket(addr, bus, dat[:length].ljust(length, b"\x00"))
          self.assertEqual(self._tx(msg), int(bus == 1 and length == 8), (bus, length))
      msg = libpanda_py.make_CANPacket(addr, 1, dat)
      msg[0].extended = 1
      self.assertEqual(self._tx(msg), 0)

  def test_temperature_query_addresses(self):
    for _, dat in self.TEMPERATURE_MSGS:
      for addr in list(range(0x700, 0x800)) + [0x18DB33F1, 0x18DA10F1]:
        self.assertEqual(self._tx(libpanda_py.make_CANPacket(addr, 1, dat)), int((addr, dat) in self.TEMPERATURE_MSGS), hex(addr))

  def test_temperature_queries_only_with_legacy_flag(self):
    for mode in (Panda.SAFETY_HYUNDAI_LEGACY, Panda.SAFETY_HYUNDAI, Panda.SAFETY_HYUNDAI_CANFD,
                 Panda.SAFETY_SILENT, Panda.SAFETY_NOOUTPUT):
      for flags in (0, Panda.FLAG_HYUNDAI_ENGINE_TEMPERATURES):
        self.assertEqual(self.safety.set_safety_hooks(mode, flags), 0)
        self.safety.init_tests()
        for addr, dat in self.TEMPERATURE_MSGS:
          expected = mode == Panda.SAFETY_HYUNDAI_LEGACY and flags == Panda.FLAG_HYUNDAI_ENGINE_TEMPERATURES
          self.assertEqual(self._tx(libpanda_py.make_CANPacket(addr, 1, dat)), int(expected), (mode, flags))

  def test_temperature_queries_blocked_on_relay_fault(self):
    self.safety.set_relay_malfunction(True)
    for addr, dat in self.TEMPERATURE_MSGS:
      self.assertEqual(self._tx(libpanda_py.make_CANPacket(addr, 1, dat)), -1)

  def test_obd_routing_follows_active_safety_configuration(self):
    # Exercise transitions to every supported mode, including clearing the flag.
    # ELM327's existing routing behavior remains unchanged.
    modes = {getattr(Panda, name) for name in dir(Panda) if name.startswith("SAFETY_")}
    for mode in modes:
      for flags in (0, 1, Panda.FLAG_HYUNDAI_ENGINE_TEMPERATURES, 0xFFFF):
        if self.safety.set_safety_hooks(mode, flags) != 0:
          continue
        expected = (mode == Panda.SAFETY_ELM327 and flags == 0) or \
                   (mode == Panda.SAFETY_HYUNDAI_LEGACY and bool(flags & Panda.FLAG_HYUNDAI_ENGINE_TEMPERATURES))
        self.assertEqual(self.safety.safety_uses_obd_can2(), expected, (mode, flags))
    self.safety.set_safety_hooks(Panda.SAFETY_SILENT, 0)
    self.assertFalse(self.safety.safety_uses_obd_can2())


class TestHyundaiLegacySafetyEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI_LEGACY, 1)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pos": gas}
    return self.packer.make_can_msg_panda("E_EMS11", 0, values, fix_checksum=checksum)


class TestHyundaiLegacySafetyHEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI_LEGACY, 2)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"CR_Vcu_AccPedDep_Pos": gas}
    return self.packer.make_can_msg_panda("E_EMS11", 0, values, fix_checksum=checksum)

class TestHyundaiLongitudinalSafety(HyundaiLongitudinalBase, TestHyundaiSafety):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0], [0x420, 0], [0x421, 0], [0x50A, 0], [0x389, 0], [0x4A2, 0], [0x38D, 0], [0x483, 0], [0x7D0, 0]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x421)}  # LKAS11, SCC12

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x421, 0)

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_kia_generic")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI, Panda.FLAG_HYUNDAI_LONG)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
      "CR_VSM_DecCmd": aeb_decel,
    }
    return self.packer.make_can_msg_panda("SCC12", self.SCC_BUS, values)

  def _fca11_msg(self, idx=0, vsm_aeb_req=False, fca_aeb_req=False, aeb_decel=0):
    values = {
      "CR_FCA_Alive": idx % 0xF,
      "FCA_Status": 2,
      "CR_VSM_DecCmd": aeb_decel,
      "CF_VSM_DecCmdAct": int(vsm_aeb_req),
      "FCA_CmdAct": int(fca_aeb_req),
    }
    return self.packer.make_can_msg_panda("FCA11", 0, values)

  def test_no_aeb_fca11(self):
    self.assertTrue(self._tx(self._fca11_msg()))
    self.assertFalse(self._tx(self._fca11_msg(vsm_aeb_req=True)))
    self.assertFalse(self._tx(self._fca11_msg(fca_aeb_req=True)))
    self.assertFalse(self._tx(self._fca11_msg(aeb_decel=1.0)))

  def test_no_aeb_scc12(self):
    self.assertTrue(self._tx(self._accel_msg(0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_req=True)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_decel=1.0)))


if __name__ == "__main__":
  unittest.main()
