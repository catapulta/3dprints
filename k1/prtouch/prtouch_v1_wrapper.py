# Open-source reconstruction of Creality's closed klippy/extras/prtouch_v1_wrapper.py
# (K1 strain-gauge Z probe, PR_VERSION 307; shipped as a Cython 0.29.32 module).
# Same class, method names/signatures, instance attributes, G-code commands and
# MCU command sequences as the binary. Python 3.8 compatible.
#
# Comments give the original .py line numbers (from the Cython error exits) and
# per-method confidence notes; build/MERGE_NOTES.md lists untested paths,
# original bugs kept on purpose and open questions.

import datetime
import logging
import math
import os
import random
import socket
import threading
import time

from . import probe

MAX_BUF_LEN = 32
MAX_PRES_CNT = 4
MAX_STEP_CNT = 4
PR_VERSION = 307

PR_ERR_CODE_PRES_READ_DATA_TIMEOUT = {'code': 'key520', 'msg': 'PR_ERR_CODE_PRES_READ_DATA_TIMEOUT: The data read interval is too large, need=11ms, actual={0}.', 'values': []}
PR_ERR_CODE_PRES_VAL_IS_CONSTANT = {'code': 'key521', 'msg': 'PR_ERR_CODE_PRES_VAL_IS_CONSTANT: The pressure data for channel={0} is incorrect. The value is constant {1}.', 'values': []}
PR_ERR_CODE_PRES_NOT_BE_SENSED = {'code': 'key522', 'msg': 'PR_ERR_CODE_PRES_NOT_BE_SENSED: The pressure data in channel={0} cannot be properly sensed.', 'values': []}
PR_ERR_CODE_PRES_LOST_RUN_DATA = {'code': 'key523', 'msg': 'PR_ERR_CODE_PRES_LOST_RUN_DATA: The pressure data is lost when the probe is over and waiting for the data to be sent back.', 'values': []}
PR_ERR_CODE_PRES_NOISE_TOO_BIG = {'code': 'key524', 'msg': 'PR_ERR_CODE_PRES_NOISE_TOO_BIG: Sensor data noise is too big, channel={0}.', 'values': []}
PR_ERR_CODE_HAVE_LOST_STEP = {'code': 'key526', 'msg': 'PR_ERR_CODE_HAVE_LOST_STEP: Z-axis motor step loss was found.', 'values': []}
PR_ERR_CODE_STEP_LOST_RUN_DATA = {'code': 'key527', 'msg': 'PR_ERR_CODE_STEP_LOST_RUN_DATA: The motor step data is lost when the probe is over and waiting for data return', 'values': []}
PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT = {'code': 'key529', 'msg': 'PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT: G28 Z try probe out of times.', 'values': []}
PR_ERR_CODE_SWAP_PIN_DETECTI = {'code': 'key532', 'msg': 'PR_ERR_CODE_SWAP_PIN_DETECTI: The synchronization pin test failed, pres_swap_pin={0}, step_swap_pin={1}.', 'values': []}
PR_ERR_CODE_PR_NOT_TRIGGER = {'code': 'key533', 'msg': 'PR_ERR_CODE_PR_NOT_TRIGGER: Prtouch Z Move But loadcell No Trigger', 'values': []}


class PRTouchEndstopWrapper:
    # lines 54-218. Body decompiled separately (FUN_0007b720, see notes).
    # Every config read matches the boot trace CFG lines (names, defaults,
    # bounds, order). The use_adc-dependent defaults are from the decompile;
    # on the K1 use_adc is False, so only the first alternative is traced.
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.steppers = []
        self.dbg_msg = {}
        self.shut_down = False
        self.run_step_prtouch_flag = False
        self.jump_probe_ready = self.has_save_sys_acc = False
        self.safe_move_z_tri_call_back, self.safe_move_z_all_cnt = None, 0
        self.mm_per_step = self.pres_tri_time = self.step_tri_time = self.pres_tri_chs = self.pres_buf_cnt = 0
        self.rdy_pos = None
        self.ovr_det_hold, self.mm_per_step = 0, None
        self.ver_step, self.ver_pres, self.ver_prth = 'V0.0', 'V0.0', 'V' + str(PR_VERSION / 100)
        self.sys_max_velocity = self.sys_max_accel = self.sys_max_z_velocity = self.sys_max_z_accel = 0
        self.step_res, self.pres_res, self.steppers = [], [], []
        self.read_swap_prtouch_cmd = self.start_step_prtouch_cmd = None
        self.write_swap_prtouch_cmd = self.read_pres_prtouch_cmd = self.start_pres_prtouch_cmd = None
        # bed_mesh appears twice in the original chained assignment (6 targets).
        self.bed_mesh = self.toolhead = self.bed_mesh = self.pheaters = self.heater_hot = self.heater_bed = None

        self.use_adc = config.getboolean('use_adc', default=False)

        self.tri_acq_ms = config.getint('tri_acq_ms', default=12 if not self.use_adc else 1, minval=1, maxval=1000)
        self.tri_send_ms = config.getint('tri_send_ms', default=10, minval=1, maxval=1000)
        self.tri_need_cnt = config.getint('tri_need_cnt', default=1, minval=1, maxval=MAX_PRES_CNT)
        self.tri_try_max_times = config.getint('tri_try_max_times', default=10, minval=0)
        self.cal_hftr_cut = config.getfloat('cal_hftr_cut', default=10, minval=0.01, maxval=100.0)
        self.cal_lftr_k1 = config.getfloat('cal_lftr_k1', default=0.85 if not self.use_adc else 0.65)

        self.tri_min_hold = config.getintlist('tri_min_hold', default=[2000, 20000] if not self.use_adc else [3, 3])
        self.tri_max_hold = config.getintlist('tri_max_hold', default=[6000, 60000] if not self.use_adc else [3072, 3072])
        self.tri_hftr_cut = config.getfloatlist('tri_hftr_cut', default=[2, 1])
        self.tri_lftr_k1 = config.getfloatlist('tri_lftr_k1', default=[0.7, 0.3] if not self.use_adc else [0.5, 0.5])

        self.show_msg = config.getboolean('show_msg', default=False)
        self.tri_wave_ip = config.get('tri_wave_ip', None)

        self.shake_cnt = config.getint('shake_cnt', default=8, minval=1, maxval=512)
        self.shake_range = config.getfloat('shake_range', default=0.5, minval=0.1, maxval=2)

        self.hot_min_temp = config.getfloat('hot_min_temp', default=140, minval=80, maxval=200)
        self.hot_max_temp = config.getfloat('hot_max_temp', default=200, minval=180, maxval=300)
        self.bed_max_temp = config.getfloat('bed_max_temp', default=60, minval=45, maxval=100)
        self.pa_clr_dis_mm = config.getfloat('pa_clr_dis_mm', default=30, minval=2, maxval=100)
        self.pa_clr_down_mm = config.getfloat('pa_clr_down_mm', default=-0.15, minval=-1, maxval=1)
        self.clr_noz_start_x = config.getfloat('clr_noz_start_x', default=0, minval=0, maxval=1000)
        self.clr_noz_start_y = config.getfloat('clr_noz_start_y', default=0, minval=0, maxval=1000)
        self.clr_noz_len_x = config.getfloat('clr_noz_len_x', default=0, minval=self.pa_clr_dis_mm + 6, maxval=1000)
        self.clr_noz_len_y = config.getfloat('clr_noz_len_y', default=0, minval=0, maxval=1000)
        self.clr_xy_spd = config.getfloat('clr_xy_spd', default=2.0, minval=0.1, maxval=10)
        self.clr_noz_quick = config.getboolean('clr_noz_quick', default=False)
        self.clr_quick_times = config.getint('clr_quick_times', default=5, minval=1, maxval=10)
        self.clr_quick_high = config.getfloat('clr_quick_high', default=2.0, minval=0.1, maxval=10)
        self.clr_xy_quick_spd = config.getfloat('clr_xy_quick_spd', default=100.0, minval=10.0, maxval=300.0)
        self.clr_quick_react_dis = config.getfloat('clr_quick_react_dis', default=5.0, minval=0.0, maxval=15.0)

        self.tri_z_down_spd = config.getfloatlist('speeds', default=[2.5, 1.0] if not self.use_adc else [10, 10])
        self.tri_z_up_spd = config.getfloat('lift_speed', self.tri_z_down_spd[0] * (2.0 if not self.use_adc else 1.0), minval=0.1, maxval=100)
        self.rdy_xy_spd = config.getfloat('rdy_xy_spd', default=200 if not self.use_adc else 300, minval=1, maxval=1000)
        self.rdy_z_spd = config.getfloat('rdy_z_spd', default=self.tri_z_up_spd, minval=1, maxval=50)
        self.acc_ctl_mm = config.getfloat('acc_ctl_mm', 0.25 if not self.use_adc else 0.5, minval=0, maxval=10)
        self.low_spd_nul = config.getint('low_spd_nul', 5, minval=1, maxval=10)
        self.send_step_duty = config.getint('send_step_duty', 16, minval=0, maxval=10)
        self.run_max_velocity = config.getfloat('run_max_velocity', default=500, minval=1, maxval=5000)
        self.run_max_accel = config.getfloat('run_max_accel', default=500, minval=1, maxval=50000)
        self.run_max_z_velocity = config.getfloat('run_max_z_velocity', default=20, minval=1, maxval=5000)
        self.run_max_z_accel = config.getfloat('run_max_z_accel', default=200, minval=1, maxval=50000)

        self.check_bed_mesh_max_err = config.getfloat('check_bed_mesh_max_err', default=0.2, minval=0.01, maxval=1)
        self.stored_profs = config.get_prefix_sections('prtouch')
        self.stored_profs = self.stored_profs[1] if len(self.stored_profs) == 2 else None

        self.need_self_check = config.getboolean('need_self_check', default=True)
        # Decompile evaluates use_adc here but both alternatives resolve to 5.
        self.bed_max_err = config.getfloat('bed_max_err', 5 if not self.use_adc else 5, minval=1, maxval=10)
        self.max_z = config.getsection('stepper_z').getfloat('position_max', default=300, minval=10, maxval=500)
        self.min_z = config.getsection('stepper_z').getfloat('position_min', default=-10, minval=-20, maxval=100)
        self.g29_down_min_z = config.getfloat('g29_down_min_z', default=25, minval=25, maxval=500)
        self.probe_min_3err = config.getfloat('probe_min_3err', default=0.1, minval=0.01, maxval=1)
        self.step_base = config.getint('step_base', 1, minval=1, maxval=10)
        self.g28_wait_cool_down = config.getboolean('g28_wait_cool_down', default=False)
        self.best_above_z = config.getfloat('best_above_z', default=3 if not self.use_adc else 4, minval=2, maxval=10)
        self.safe_z_dead_dis = config.getfloat('safe_z_dead_dis', default=2, minval=0, maxval=10)

        self.fan_heat_min_spd = config.getfloat('fan_heat_min_spd', default=0.3, minval=0, maxval=255)
        self.fan_heat_max_spd = config.getfloat('fan_heat_max_spd', default=1.0, minval=0, maxval=255)

        self.gcode = self.printer.lookup_object('gcode')
        self.lost_step_dis = config.getfloat('lost_step_dis', default=0.5, minval=0, maxval=10)
        self.tilt_corr_dis = config.getfloat('tilt_corr_dis', default=0, minval=-0.1, maxval=0.1)

        self.noz_ex_com = config.getfloat('noz_ex_com', default=0.0, minval=0, maxval=1)
        self.sys_time_duty = config.getfloat('sys_time_duty', default=0.001, minval=0.00001, maxval=0.01)
        self.is_corexz = str(config.getsection('printer').get('kinematics')) == 'corexz'

        self.shake_sample_num = config.getint('shake_sample_num', default=100, minval=1, maxval=1000)
        self.shake_z_velocity = config.getint('shake_z_velocity', default=100, minval=1, maxval=1000)
        self.shake_z_accel = config.getint('shake_z_accel', default=500, minval=1, maxval=10000)
        self.shake_num = config.getint('shake_num', default=20, minval=1, maxval=100)
        self.check_std_max = config.getint('check_std_max', default=self.tri_min_hold[0], minval=1, maxval=8000)
        self.check_std_mim = config.getint('check_std_mim', default=250, minval=1, maxval=8000)

        self.step_swap_pin = config.get('step_swap_pin')
        self.pres_swap_pin = config.get('pres_swap_pin')

        self.pres_cnt = config.getint('pres_cnt', 1, 1, MAX_PRES_CNT)
        self.pres_clk_pins, self.pres_sdo_pins, self.pres_adc_pins = [], [], []
        for i in range(self.pres_cnt):
            if self.use_adc:
                self.pres_adc_pins.append(config.get('pres%d_adc_pins' % i))
            else:
                self.pres_clk_pins.append(config.get('pres%d_clk_pins' % i))
                self.pres_sdo_pins.append(config.get('pres%d_sdo_pins' % i))

        self.z_step_pins, self.z_dir_pins = [], []
        for stepper in ['stepper_z', 'stepper_x' if self.is_corexz else 'stepper_z1', 'stepper_z2', 'stepper_z3']:
            if config.has_section(stepper):
                stepper_config = config.getsection(stepper)
                self.z_step_pins.append(stepper_config.get('step_pin'))
                self.z_dir_pins.append(stepper_config.get('dir_pin'))

        self.ppins = self.printer.lookup_object('pins')
        self.step_mcu, self.pres_mcu = self.ppins.chips['mcu'], self.ppins.chips['mcu']
        if self.step_swap_pin and self.pres_swap_pin:
            self.step_mcu = self.ppins.parse_pin(self.step_swap_pin, True, True)['chip']
            self.pres_mcu = self.ppins.parse_pin(self.pres_swap_pin, True, True)['chip']
        self.step_oid, self.pres_oid = self.step_mcu.create_oid(), self.pres_mcu.create_oid()

        self.step_mcu.register_config_callback(self._build_step_config)
        self.pres_mcu.register_config_callback(self._build_pres_config)

        self.gcode.register_command('TEST_PRTH', self.cmd_TEST_PRTH, desc=self.cmd_TEST_PRTH_help)
        self.gcode.register_command('READ_PRES', self.cmd_READ_PRES, desc=self.cmd_READ_PRES_help)
        self.gcode.register_command('TEST_SWAP', self.cmd_TEST_SWAP, desc=self.cmd_TEST_SWAP_help)
        self.gcode.register_command('DEAL_AVGS', self.cmd_DEAL_AVGS, desc=self.cmd_DEAL_AVGS_help)
        self.gcode.register_command('TRIG_TEST', self.cmd_TRIG_TEST, desc=self.cmd_TRIG_TEST_help)
        self.gcode.register_command('CHECK_BED_MESH', self.cmd_CHECK_BED_MESH, desc=self.cmd_CHECK_BED_MESH_help)
        self.gcode.register_command('PRTOUCH_READY', self.cmd_PRTOUCH_READY, desc=self.cmd_PRTOUCH_READY_help)
        self.gcode.register_command('NOZZLE_CLEAR', self.cmd_NOZZLE_CLEAR, desc=self.cmd_NOZZLE_CLEAR_help)
        self.gcode.register_command('SAFE_DOWN_Z', self.cmd_SAFE_DOWN_Z, desc=self.cmd_SAFE_DOWN_Z_help)
        self.gcode.register_command('SAFE_MOVE_Z', self.cmd_SAFE_MOVE_Z, desc=self.cmd_SAFE_MOVE_Z_help)
        self.gcode.register_command('ACCURATE_HOME_Z', self.cmd_ACCURATE_HOME_Z, desc=self.cmd_ACCURATE_HOME_Z_help)
        self.gcode.register_command('SELF_CHECK_PRTOUCH', self.cmd_SELF_CHECK_PRTOUCH, desc=self.cmd_SELF_CHECK_PRTOUCH_help)
        self.gcode.register_command('START_STEP_PRTOUCH', self.cmd_START_STEP_PRTOUCH, desc=self.cmd_START_STEP_PRTOUCH_help)
        self.gcode.register_command('TRIG_BED_TEST', self.cmd_TRIG_BED_TEST, desc=self.cmd_TRIG_BED_TEST_help)

        self.step_mcu.register_response(self._handle_step_debug_prtouch, 'debug_prtouch', self.step_oid)
        self.step_mcu.register_response(self._handle_result_run_step_prtouch, 'result_run_step_prtouch', self.step_oid)

        self.pres_mcu.register_response(self._handle_pres_debug_prtouch, 'debug_prtouch', self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_run_pres_prtouch, 'result_run_pres_prtouch', self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_read_pres_prtouch, 'result_read_pres_prtouch', self.pres_oid)

        self.v_sd = self.printer.lookup_object('virtual_sdcard')  # tuple const @135b04, name inferred from trace type
        self.run_sta_status = 1
        self.z_set_status = False
        self.nozzle_clear_z_out_of_range = False
        self.After_G28Z_skip_bed_tilt_flag = False
        self.trig_info = [-1, -1, -1, -1]
        self.Tri_bed_test_z = []
        self.save_pres_file = None

    # lines 221-266
    def _build_step_config(self):
        self.bed_mesh = self.printer.lookup_object('bed_mesh')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.probe = self.printer.lookup_object('probe')
        self.pheaters = self.printer.lookup_object('heaters')
        self.heater_hot = self.printer.lookup_object('extruder').heater
        self.heater_bed = self.printer.lookup_object('heater_bed').heater

        if 'heater_fan' in self.printer.objects:
            self.fan_heat_obj = self.printer.lookup_object('heater_fan')
        if 'fan' in self.printer.objects:
            self.fan_mode_obj = self.printer.lookup_object('fan')

        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max

        self.home_x = min_x + (max_x - min_x) / 2
        self.home_y = min_y + (max_y - min_y) / 2

        if self.clr_noz_start_x <= 0 or self.clr_noz_start_y <= 0 or self.clr_noz_len_x <= 0 or self.clr_noz_len_y <= 0:
            self.clr_noz_start_x = (max_x - min_x) * 1 / 3 + min_x
            self.clr_noz_start_y = max_y - 6
            self.clr_noz_len_x = (max_x - min_x) * 1 / 3
            self.clr_noz_len_y = 5

        random.seed(time.time())
        self.rdy_pos = [[min_x, min_y, self.bed_max_err], [min_x, max_y, self.bed_max_err],
                        [max_x, max_y, self.bed_max_err], [max_x, min_y, self.bed_max_err]]

        self.step_mcu.add_config_cmd('config_step_prtouch oid=%d step_cnt=%d swap_pin=%s sys_time_duty=%u' % (
            self.step_oid, len(self.z_step_pins), self.ppins.parse_pin(self.step_swap_pin, True, True)['pin'],
            int(self.sys_time_duty * 100000)))

        for i in range(len(self.z_step_pins)):
            step_par = self.ppins.parse_pin(self.z_step_pins[i], True, True)
            dir_par = self.ppins.parse_pin(self.z_dir_pins[i], True, True)
            if self.is_corexz:
                if i == 0:
                    self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (
                        self.step_oid, 0, dir_par['pin'], step_par['pin'], not dir_par['invert'], step_par['invert']))
                else:
                    self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (
                        self.step_oid, i, dir_par['pin'], step_par['pin'], dir_par['invert'], step_par['invert']))
            else:
                self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (
                    self.step_oid, i, dir_par['pin'], step_par['pin'], dir_par['invert'], step_par['invert']))

        self.read_swap_prtouch_cmd = self.step_mcu.lookup_query_command('read_swap_prtouch oid=%c', 'result_read_swap_prtouch oid=%c sta=%c', oid=self.step_oid)
        self.start_step_prtouch_cmd = self.step_mcu.lookup_command('start_step_prtouch oid=%c dir=%c send_ms=%c step_cnt=%u step_us=%u acc_ctl_cnt=%u low_spd_nul=%c send_step_duty=%c auto_rtn=%c', cq=None)
        self.manual_get_steps_cmd = self.step_mcu.lookup_query_command('manual_get_steps oid=%c index=%c', 'result_manual_get_steps oid=%c index=%c tri_time=%u tick0=%u tick1=%u tick2=%u tick3=%u step0=%u step1=%u step2=%u step3=%u', oid=self.step_oid)

    # lines 269-286
    def _build_pres_config(self):
        self.pres_mcu.add_config_cmd('config_pres_prtouch oid=%d use_adc=%d pres_cnt=%d swap_pin=%s sys_time_duty=%u' % (
            self.pres_oid, self.use_adc, self.pres_cnt, self.ppins.parse_pin(self.pres_swap_pin, True, True)['pin'],
            int(self.sys_time_duty * 100000)))
        for i in range(self.pres_cnt):
            if self.use_adc:
                adc_par = self.ppins.parse_pin(self.pres_adc_pins[i], True, True)
                self.pres_mcu.add_config_cmd('add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (
                    self.pres_oid, i, adc_par['pin'], adc_par['pin']))
            else:
                clk_par = self.ppins.parse_pin(self.pres_clk_pins[i], True, True)
                sdo_par = self.ppins.parse_pin(self.pres_sdo_pins[i], True, True)
                self.pres_mcu.add_config_cmd('add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (
                    self.pres_oid, i, clk_par['pin'], sdo_par['pin']))

        self.write_swap_prtouch_cmd = self.pres_mcu.lookup_query_command('write_swap_prtouch oid=%c sta=%c', 'resault_write_swap_prtouch oid=%c', oid=self.pres_oid)
        self.read_pres_prtouch_cmd = self.pres_mcu.lookup_command('read_pres_prtouch oid=%c acq_ms=%u cnt=%u', cq=None)
        self.start_pres_prtouch_cmd = self.pres_mcu.lookup_command('start_pres_prtouch oid=%c tri_dir=%c acq_ms=%c send_ms=%c need_cnt=%c tri_hftr_cut=%u tri_lftr_k1=%u min_hold=%u max_hold=%u', cq=None)
        self.deal_avgs_prtouch_cmd = self.pres_mcu.lookup_query_command('deal_avgs_prtouch oid=%c base_cnt=%c', 'result_deal_avgs_prtouch oid=%c ch0=%i ch1=%i ch2=%i ch3=%i', oid=self.pres_oid)
        self.manual_get_pres_cmd = self.pres_mcu.lookup_query_command('manual_get_pres oid=%c index=%c', 'resault_manual_get_pres oid=%c index=%c tri_time=%u tri_chs=%c buf_cnt=%u tick_0=%u ch0_0=%i ch1_0=%i ch2_0=%i ch3_0=%i tick_1=%u ch0_1=%i ch1_1=%i ch2_1=%i ch3_1=%i', oid=self.pres_oid)

    # lines 290-292
    def _handle_step_debug_prtouch(self, params):
        self.ver_step = 'V' + str(params['version'] / 100)

    # lines 295-310
    def _handle_result_run_step_prtouch(self, params):
        self.step_tri_time = params['tri_time'] / 10000.0
        for i in range(4):
            step = {'tick': params['tick%d' % i] / 10000.0, 'step': params['step%d' % i], 'index': params['index']}
            self.step_res.append(step)
        if self.safe_move_z_tri_call_back != None and len(self.step_res) == MAX_BUF_LEN:
            tri_dis = (self.safe_move_z_all_cnt - self.step_res[-1]['step']) * self.mm_per_step
            self.safe_move_z_tri_call_back(tri_dis)
            self.safe_move_z_tri_call_back = None
            self.print_msg('SAFE_MOVE_Z', 'tri_dis = %f' % tri_dis, True)
        if self.run_sta_status == 0 and self.safe_move_z_tri_call_back != None:
            tri_dis = (self.safe_move_z_all_cnt - self.step_res[-1]['step']) * self.mm_per_step
            self.safe_move_z_tri_call_back(tri_dis)
            self.safe_move_z_tri_call_back = None
            self.run_sta_status = 1

    # lines 313-315
    def _handle_pres_debug_prtouch(self, params):
        self.ver_pres = 'V' + str(params['version'] / 100)

    # lines 318-325
    def _handle_result_run_pres_prtouch(self, params):
        self.pres_tri_time = params['tri_time'] / 10000.0
        self.pres_tri_chs = params['tri_chs']
        self.pres_buf_cnt = params['buf_cnt']
        for i in range(2):
            pres = {'tick': params['tick_%d' % i] / 10000.0, 'ch0': params['ch0_%d' % i], 'ch1': params['ch1_%d' % i],
                    'ch2': params['ch2_%d' % i], 'ch3': params['ch3_%d' % i], 'index': params['index']}
            self.pres_res.append(pres)

    # lines 328-330
    def _handle_result_read_pres_prtouch(self, params):
        self.pres_res.append(params)

    # line 335-336
    def get_mcu(self):
        return self.step_mcu

    # line 338-339
    def get_position_endstop(self):
        return 0

    # lines 341-345. Not traced (the K1 Z rail uses a TMC virtual endstop, so
    # nothing calls add_stepper on the wrapper). mm_per_step is computed from
    # steppers[0] BEFORE the membership test / append, exactly as decompiled:
    # with an empty self.steppers this raises IndexError, as the original does.
    def add_stepper(self, stepper):
        self.mm_per_step = self.steppers[0].get_step_dist() * self.step_base
        if stepper in self.steppers:
            return
        self.steppers.append(stepper)

    # line 347-348
    def get_steppers(self):
        return list(self.steppers)

    # line 350-351
    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        return True

    # line 353-354 (arg name home_end_time from kwarg table)
    def home_wait(self, home_end_time):
        return True

    # line 356-357
    def multi_probe_begin(self):
        pass

    # line 359-360
    def multi_probe_end(self):
        pass

    # line 362-363
    def query_endstop(self, print_time):
        return False

    # line 365-366
    def probe_prepare(self, hmove):
        pass

    # line 368-369 (body returns None; not yet checked for side effects beyond arg parsing)
    def probe_finish(self, hmove):
        pass

    # lines 373-385. Signature from the argname array (self, title, msg,
    # force) with force defaulting to False.
    # Line 376 emits no code (comment or blank in the source). The UDP debug
    # sink is only reached when the message was shown (show_msg or force).
    def print_msg(self, title, msg, force=False):
        logging.info('[%s] %s', title, msg)
        if self.show_msg or force:
            if title != 'SHOW_WAVE':
                self.gcode.respond_info('[' + title + ']' + msg)
            if self.tri_wave_ip is not None:
                if title not in self.dbg_msg:
                    self.dbg_msg[title] = len(self.dbg_msg)
                ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                ss.sendto((str(self.dbg_msg[title]) + '$' + title + '$' + time.strftime('%H:%M:%S ')
                           + '$' + msg + '\n').encode(), (str(self.tri_wave_ip), 21021))
                ss.close()

    # lines 388-395. Defaults lent=32, pt_cnt=3, force=False from the wrapper's
    # default slots. Each element is formatted with '%.' + ('%df, ' % pt_cnt),
    # so the string keeps a trailing ', ' before ']'.
    def print_ary(self, title, ary, lent=32, pt_cnt=3, force=False):
        st = '['
        for i in range(len(ary) - lent, len(ary)):
            st = st + ('%.' + '%df, ' % pt_cnt) % ary[i]
        st += ']'
        logging.info('[%s] %s', title, st)
        if self.show_msg or force:
            self.print_msg(title, st, force)

    # lines 398-415. title (default is the str 'None', not the None object)
    # is accepted but unused.
    def print_res(self, title='None'):
        step_tick, step_data = [], []
        for i in range(len(self.step_res)):
            step_tick.append(self.step_res[i]['tick'])
            step_data.append(self.step_res[i]['step'])
        self.print_ary('STEP_TICK', step_tick, len(step_tick))
        self.print_ary('STEP_DATA', step_data, len(step_data))

        pres_tick, pres_data = [], [[], [], [], []]
        for i in range(len(self.pres_res)):
            pres_tick.append(self.pres_res[i]['tick'])
            for j in range(self.pres_cnt):
                pres_data[j].append(self.pres_res[i]['ch%d' % j])

        self.print_ary('PRES_TICK', pres_tick, len(pres_tick))
        for i in range(self.pres_cnt):
            self.send_wave_tri(i, pres_data[i])
            self.print_ary('PRES_CH%d' % i, pres_data[i], len(pres_data[i]))

    # lines 420-441. Not traced (no error was raised in any trace).
    # vals default is a single shared list created at module init (mutable
    # default kept as in the original). err_code is one of the module-level
    # PR_ERR_CODE_* dicts and is mutated in place.
    # Lines 426-428: only the constant 5 (local here named run_dis) is
    # assigned; the original local name is not recoverable from the decompile.
    def ck_and_raise_error(self, ck, err_code, vals=[]):
        if not ck:
            return
        self.print_msg('RAISE_ERROR', 'Start Raise Error')
        self.enable_steps()
        run_dis = 5
        step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(run_dis, 10)
        self.step_res = []
        self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt,
                                          self.low_spd_nul, self.send_step_duty, 0])
        t_start = time.time()
        while (time.time() - t_start) < (run_dis / 10 + 10) and len(self.step_res) != MAX_BUF_LEN:
            self.delay_s(0.01)
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
        self.disable_steps()
        err_code['values'] = vals
        self.print_msg('RAISE_ERROR', str(err_code), True)
        raise self.printer.command_error(str(err_code))

    # lines 444-450. No tri_wave_ip guard here (unlike the open-source
    # prtouch.py); the UDP send happens inside print_msg. The trailing field
    # is trig_info[ch] (set by cal_tri_data), which is then reset to -1.
    def send_wave_tri(self, ch, ary):
        msg = '%d' % ch
        for i in range(len(ary)):
            msg += ',%d' % ary[i]
        msg += ',%d' % self.trig_info[ch]
        self.print_msg('SHOW_WAVE', msg)
        self.trig_info[ch] = -1

    # lines 453-458. The only direct toolhead call is get_last_move_time();
    # the traced TH flush_step_generation and STP generate_steps lines are
    # made inside it / inside the reactor pause.
    def delay_s(self, delay_s):
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        if self.printer.is_shutdown():
            return
        self.toolhead.get_last_move_time()
        reactor.pause(eventtime + delay_s)

    # get_linear2: original lines 461-466. Confidence: high (decompile matches the
    # open-source prtouch.py _get_linear2 exactly, including the 0.001 threshold).
    def get_linear2(self, p1, p2, po, is_base_x):
        if (math.fabs(p1[0] - p2[0]) < 0.001 and is_base_x) or (math.fabs(p1[1] - p2[1]) < 0.001 and not is_base_x):
            return po
        a = (p2[2] - p1[2]) / ((p2[0] - p1[0]) if is_base_x else (p2[1] - p1[1]))
        b = p1[2] - (p1[0] if is_base_x else p1[1]) * a
        po[2] = a * (po[0] if is_base_x else po[1]) + b
        return po

    # lines 470-473. Homes all axes once if any axis is unhomed (kinematics
    # limits min > max means unhomed).
    def ck_g28ed(self):
        for i in range(3):
            if self.toolhead.kin.limits[i][0] > self.toolhead.kin.limits[i][1]:
                self.gcode.run_script_from_command('G28')
                break

    # lines 476-494. Signature from the kwarg table at 0x11f474 (default
    # load_sys=True). Writes the toolhead / kinematics attributes directly;
    # there is no call into the toolhead to recompute junction deviation.
    def set_step_par(self, load_sys=True):
        if load_sys:                                                      # 477
            self.toolhead.max_velocity = self.sys_max_velocity            # 478
            self.toolhead.max_accel = self.sys_max_accel                  # 479
            self.toolhead.kin.max_z_velocity = self.sys_max_z_velocity    # 480
            self.toolhead.kin.max_z_accel = self.sys_max_z_accel          # 481
            self.has_save_sys_acc = False                                 # 482
            return
        if self.has_save_sys_acc:                                         # 484
            return
        self.sys_max_velocity = self.toolhead.max_velocity                # 485
        self.sys_max_accel = self.toolhead.max_accel                      # 486
        self.sys_max_z_velocity = self.toolhead.kin.max_z_velocity        # 487
        self.sys_max_z_accel = self.toolhead.kin.max_z_accel              # 488
        self.toolhead.max_velocity = self.run_max_velocity                # 490
        self.toolhead.max_accel = self.run_max_accel                      # 491
        self.toolhead.kin.max_z_velocity = self.run_max_z_velocity        # 492
        self.toolhead.kin.max_z_accel = self.run_max_z_accel              # 493
        self.has_save_sys_acc = True                                      # 494

    # lines 497-507. Line 503 binds is_motor_enabled() before the test on 504.
    def enable_steps(self):
        self.print_msg('ENABLE_STEPS', 'Start enable_steps()...')         # 498
        for stepper in self.toolhead.get_kinematics().get_steppers():     # 499
            print_time = self.toolhead.get_last_move_time()               # 500
            stepper_enable = self.printer.lookup_object('stepper_enable')  # 501
            enable = stepper_enable.lookup_enable(stepper.get_name())     # 502
            is_enabled = enable.is_motor_enabled()                        # 503
            if not is_enabled:                                            # 504
                enable.motor_enable(print_time)                           # 505
                self.toolhead.dwell(0.1)                                  # 506
                self.delay_s(0.5)                                         # 507

    # lines 510-520. Mirror of enable_steps.
    def disable_steps(self):
        self.print_msg('DISABLE_STEPS', 'Start disable_steps()...')       # 511
        for stepper in self.toolhead.get_kinematics().get_steppers():     # 512
            print_time = self.toolhead.get_last_move_time()               # 513
            stepper_enable = self.printer.lookup_object('stepper_enable')  # 514
            enable = stepper_enable.lookup_enable(stepper.get_name())     # 515
            is_enabled = enable.is_motor_enabled()                        # 516
            if is_enabled:                                                # 517
                enable.motor_disable(print_time)                          # 518
                self.toolhead.dwell(0.1)                                  # 519
                self.delay_s(0.5)                                         # 520

    # lines 523-530. Signature from the kwarg table at 0x11f480.
    # Goes through G-code (run_script_from_command), not toolhead.move.
    def move(self, pos, speed, wait=True):
        if self.shut_down:                                                # 525
            return
        if len(pos) < 3:                                                  # 526
            gcmd = 'G1 F%d X%.3f Y%.3f' % (speed * 60, pos[0], pos[1])
        else:
            gcmd = 'G1 F%d X%.3f Y%.3f Z%.3f' % (speed * 60, pos[0], pos[1], pos[2])
        self.print_msg('PRTOUCH_MOVE', 'Start Move, gcmd=' + gcmd)        # 527
        self.gcode.run_script_from_command(gcmd)                          # 528
        if wait:                                                          # 529
            self.toolhead.wait_moves()                                    # 530

    # lines 534-547. prtouch_v2_fan (on klippy:ready) moves this bound method to
    # self.set_fan_speed__ and installs set_fan_speed(self, fan_name='None',
    # fan_spd=0.0) that clamps fan_spd to max_speed for fan_name == 'fan'
    # and then calls set_fan_speed__. Callers must go through
    # self.set_fan_speed with positional (fan_name, fan_spd).
    def set_fan_speed(self, fan_name='None', fan_spd=0.0):
        self.print_msg('SET_FAN_SPEED', 'fan_name=%s, fan_spd=%f' % (fan_name, fan_spd))
        if fan_name == 'heater_fan':
            for obj in self.printer.objects:
                if 'heater_fan' in obj:
                    self.printer.objects[obj].fan.set_speed_from_command(fan_spd)
                    break
        elif fan_name == 'fan':
            if 'fan' in self.printer.objects:
                self.printer.lookup_object('fan').fan.set_speed_from_command(fan_spd)
            else:
                self.gcode.run_script_from_command('M106 P0 S%d' % (fan_spd * 255))
                self.gcode.run_script_from_command('M106 P2 S%d' % (fan_spd * 255))

    # lines 550-555. Defaults wait=False, err=5 (wrapper init values).
    # set_temperature's third positional arg is the literal False.
    def set_hot_temps(self, temp, wait=False, err=5):
        self.print_msg('SET_HOT_TEMPS', 'temp=%.2f, wait=%d, err=%f' % (temp, wait, err))  # 551
        self.pheaters.set_temperature(self.heater_hot, temp, False)       # 552
        if wait:                                                          # 553
            while not self.shut_down and abs(self.heater_hot.target_temp - self.heater_hot.smoothed_temp) > err \
                    and self.heater_hot.target_temp > 0:                  # 554
                self.delay_s(0.1)                                         # 555

    # lines 558-564. Bed goes through M140 (G-code), hotend through pheaters.
    # Line 560 emits no code (blank or comment in the original).
    def set_bed_temps(self, temp, wait=False, err=5):
        self.print_msg('SET_BED_TEMPS', 'temp=%.2f, wait=%d, err=%f' % (temp, wait, err))  # 559
        self.gcode.run_script_from_command('M140 S%d' % temp)             # 561
        if wait:                                                          # 562
            while not self.shut_down and abs(self.heater_bed.target_temp - self.heater_bed.smoothed_temp) > err \
                    and self.heater_bed.target_temp > 0:                  # 563
                self.delay_s(0.1)                                         # 564

    # shake_motor: original lines 567-578 (body FUN_0005de20).
    # Confidence: high. Every G1 line, the F value, the queue-wait and the final move were
    # checked against trace_G29 (cnt=2.0) and trace_AHZ (cnt=8).
    # Unlike the open-source prtouch.py version, the K1 binary does not swap toolhead
    # velocity/accel limits here; F comes from kin.max_z_velocity.
    # The queue test is `len(queue) > 4` in the binary (loop breaks when size < 5); the
    # source may have been written `>= 5`, which is identical for ints.
    def shake_motor(self, cnt):
        self.print_msg('SHAKE_MOTOR', 'cnt=%d' % cnt)                               # 568
        now_pos = self.toolhead.get_position()                                      # 569
        max_z_velocity = self.toolhead.kin.max_z_velocity                           # 570
        for i in range(int(cnt / 2)):                                               # 571
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (
                now_pos[0] - self.shake_range, now_pos[1] - self.shake_range,
                now_pos[2] - self.shake_range / 2, int(60 * max_z_velocity * 0.25)))  # 572
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (
                now_pos[0] + self.shake_range, now_pos[1] - self.shake_range,
                now_pos[2] + self.shake_range / 2, int(60 * max_z_velocity * 0.25)))  # 573
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (
                now_pos[0] + self.shake_range, now_pos[1] + self.shake_range,
                now_pos[2] - self.shake_range / 2, int(60 * max_z_velocity * 0.25)))  # 574
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (
                now_pos[0] - self.shake_range, now_pos[1] + self.shake_range,
                now_pos[2] + self.shake_range / 2, int(60 * max_z_velocity * 0.25)))  # 575
            while len(self.toolhead.move_queue.queue) > 4:                          # 576
                self.delay_s(0.01)                                                  # 577
        self.move(now_pos, self.rdy_z_spd)                                          # 578

    # quick_shake_motor: original lines 581-603 (body FUN_0005c004).
    # Confidence: high for every executed statement (trace_G28 / trace_AHZ, cnt=20).
    # Lines 583, 586, 589, 593 and 597-599 carry no error exit; they are taken to be blank
    # lines or comments. Line 593 sits inside the `if`; if it held a statement it could only
    # be one without any Python-level failure path (none is visible in the decompile).
    def quick_shake_motor(self, cnt):
        now_pos = self.toolhead.get_position()                                      # 582
        max_z_velocity = self.toolhead.kin.max_z_velocity                           # 584
        max_z_accel = self.toolhead.kin.max_z_accel                                 # 585
        self.toolhead.kin.max_z_velocity = self.shake_z_velocity                    # 587
        self.toolhead.kin.max_z_accel = self.shake_z_accel                          # 588
        self.print_msg('SHAKE_MOTOR', 'shake_cnt={} shake_z_velocity={} shake_z_accel:{} shake_range:{} max_z_v:{} max_z_accel:{}'.format(
            cnt, self.shake_z_velocity, self.shake_z_accel, self.shake_range, max_z_velocity, max_z_accel))  # 590
        for i in range(cnt):                                                        # 591
            if i % 2 == 0:                                                          # 592
                self.gcode.run_script_from_command('G1 Z%.2f F%d' % (now_pos[2], int(60 * self.shake_z_velocity)))  # 594
            else:
                self.gcode.run_script_from_command('G1 Z%.2f F%d' % (now_pos[2] + self.shake_range, int(60 * self.shake_z_velocity)))  # 596
        self.toolhead.kin.max_z_velocity = max_z_velocity                           # 600
        self.toolhead.kin.max_z_accel = max_z_accel                                 # 601
        self.move(now_pos, self.rdy_z_spd)                                          # 603

    # lines 608-621. Re-requests every missing 4-sample block of step_res.
    # The final test is `len != MAX_BUF_LEN` -> False else True.
    def ck_and_manual_get_step(self):
        if len(self.step_res) == MAX_BUF_LEN:                             # 609
            return True                                                   # 610
        self.print_msg('CK_AND_MANUAL_GET_STEP', 'need={}, recv={}, step_res={}'.format(
            MAX_BUF_LEN, len(self.step_res), self.step_res))              # 611
        for i in range(0, MAX_BUF_LEN, 4):                                # 612
            if len(self.step_res) > i and self.step_res[i]['index'] == i:  # 613
                continue                                                  # 614
            self.print_msg('CK_AND_MANUAL_GET_STEP', 'Re trans index=%d' % i)  # 615
            params = self.manual_get_steps_cmd.send([self.step_oid, i])   # 616
            self.step_tri_time = params['tri_time'] / 10000.0             # 617
            for j in range(4):                                            # 618
                tmp = {'tick': params['tick%d' % j] / 10000.0, 'step': params['step%d' % j],
                       'index': params['index']}                          # 619
                self.step_res.insert(i + j, tmp)                          # 620
        if len(self.step_res) != MAX_BUF_LEN:                             # 621
            return False
        return True

    # lines 628-643. Same as ck_and_manual_get_step for the 2-sample pressure
    # blocks. NOTE: the binary sends self.step_oid (not pres_oid) to
    # manual_get_pres_cmd (GOT slot 0x135f28 = n_step_oid); kept as-is.
    def ck_and_manual_get_pres(self):
        if len(self.pres_res) == MAX_BUF_LEN:                             # 629
            return True                                                   # 630
        self.print_msg('CK_AND_MANUAL_GET_PRES', 'need={}, recv={}, pres_res:{}'.format(
            MAX_BUF_LEN, len(self.pres_res), self.pres_res))              # 631
        for i in range(0, MAX_BUF_LEN, 2):                                # 632
            if len(self.pres_res) > i and self.pres_res[i]['index'] == i:  # 633
                continue                                                  # 634
            self.print_msg('CK_AND_MANUAL_GET_PRES', 'Re trans index=%d' % i)  # 635
            params = self.manual_get_pres_cmd.send([self.step_oid, i])    # 636
            self.pres_tri_time = params['tri_time'] / 10000.0             # 637
            self.pres_tri_chs = params['tri_chs']                         # 638
            self.pres_buf_cnt = params['buf_cnt']                         # 639
            for j in range(2):                                            # 640
                tmp = {'tick': params['tick_%d' % j] / 10000.0,
                       'ch0': params['ch0_%d' % j], 'ch1': params['ch1_%d' % j],
                       'ch2': params['ch2_%d' % j], 'ch3': params['ch3_%d' % j],
                       'index': params['index']}                          # 641
                self.pres_res.insert(i + j, tmp)                          # 642
        if len(self.pres_res) != MAX_BUF_LEN:                             # 643
            return False
        return True

    # get_valid_ch: original lines 649-659. Confidence: high.
    # Invalid channels (bit not set in pres_tri_chs) get the sentinel max_x * max_y * 2.
    def get_valid_ch(self):
        now_pos = self.toolhead.get_position()
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        chs = []
        chs.append(math.sqrt((now_pos[0] - min_x) ** 2 + (now_pos[1] - min_y) ** 2) if self.pres_tri_chs & 1 else max_x * max_y * 2)
        chs.append(math.sqrt((now_pos[0] - max_x) ** 2 + (now_pos[1] - min_y) ** 2) if self.pres_tri_chs & 2 else max_x * max_y * 2)
        chs.append(math.sqrt((now_pos[0] - min_x) ** 2 + (now_pos[1] - max_y) ** 2) if self.pres_tri_chs & 4 else max_x * max_y * 2)
        chs.append(math.sqrt((now_pos[0] - max_x) ** 2 + (now_pos[1] - max_y) ** 2) if self.pres_tri_chs & 8 else max_x * max_y * 2)
        best_ch = chs.index(min(chs))
        self.print_msg('VALID_CH', 'Tri_mark=%d best_ch=%d Chs=' % (self.pres_tri_chs, best_ch) + '  ' + str(chs))
        return best_ch, chs

    # cal_tri_data: original lines 672-758. Confidence: high for structure and arithmetic
    # order (each float op order taken from the Cython call sequence); validated by
    # recon/check_core.py against every traced call (see notes).
    def cal_tri_data(self, start_step, start_pos_z, step_res, pres_res, oft_z=0):
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        best_ch, chs = self.get_valid_ch()
        out_mms = []
        for ch in range(len(chs)):
            if chs[ch] == max_x * max_y * 2:
                continue
            pres_tick = []
            pres_val = []
            for i in range(len(pres_res)):
                pres_tick.append(pres_res[i]['tick'] - self.pres_tri_time)
                pres_val.append(pres_res[i]['ch%d' % ch])
            step_tick = []
            step_val = []
            for i in range(len(step_res)):
                step_tick.append(step_res[i]['tick'] - self.step_tri_time)
                step_val.append(step_res[i]['step'])
            if not self.use_adc:
                # L702-L720: outlier clamp then first-order high-pass filter
                avg = sum(pres_val) / len(pres_val)
                s = 0
                for i in range(len(pres_val)):
                    s += math.pow(pres_val[i] - avg, 2)
                std = math.sqrt(s / len(pres_val))
                for i in range(1, len(pres_val) - 2):
                    if math.fabs(pres_val[i] - avg) / std > 2:
                        pres_val[i] = pres_val[i - 1]
                rc = 1.0 / 2.0 / math.pi / self.cal_hftr_cut  # compiled as 0.5 / math.pi / cal_hftr_cut
                alpha = rc / (rc + 1.0 / (1000 / self.tri_acq_ms))
                tmp = [v for v in pres_val]
                out = [0]
                for i in range(1, len(tmp)):
                    out.append((tmp[i] - tmp[i - 1] + out[-1]) * alpha)
                pres_val = out
            # L722-L723: first-order low-pass filter
            for i in range(1, len(pres_val)):
                pres_val[i] = pres_val[i - 1] * (1 - self.cal_lftr_k1) + pres_val[i] * self.cal_lftr_k1
            self.print_ary('CAL_TRI_DATA_FILTER_CH%d' % ch, pres_val, len(pres_val), 0)
            # L727-L736: normalise, rotate by the end-to-end slope, pick the minimum
            vals = [v for v in pres_val]
            min_v = min(vals)
            max_v = max(vals)
            for i in range(len(vals)):
                vals[i] = (vals[i] - min_v) / (max_v - min_v)
            angle = math.atan((vals[-1] - vals[0]) / len(vals))
            sin_a = math.sin(-angle)
            cos_a = math.cos(-angle)
            for i in range(len(vals)):
                vals[i] = (i - 0) * sin_a + (vals[i] - 0) * cos_a + 0
            pres_tri_index = vals.index(min(vals))
            pres_tri_tick = pres_tick[pres_tri_index]
            # L739-L748: map the pressure trigger tick onto the step timeline
            step_tri_index = MAX_BUF_LEN - 1
            step_tri_tick = step_tick[step_tri_index]
            for i in range(len(step_tick) - 1):
                if (step_tick[i] <= pres_tri_tick <= step_tick[i + 1] or step_tick[i] == pres_tri_tick) and step_val[i] != 0:
                    step_tri_index = i
                    step_tri_tick = step_tick[i]
                    break
            tri_step = step_val[-1]
            if 0 < step_tri_index < len(step_res) - 1:
                tri_step = step_val[step_tri_index] + (step_val[step_tri_index + 1] - step_val[step_tri_index]) * (pres_tri_tick - step_tri_tick) / (step_tick[step_tri_index + 1] - step_tick[step_tri_index])
            # L749-L755
            cal_tri_z = (start_step - tri_step) * self.mm_per_step
            out_mm = start_pos_z - cal_tri_z + oft_z
            self.print_msg('TRI_PRES_MSG', 'start_pres={:.3f}, tri_pres={:.3f}, end_pres={:.3f}, pres_tri_index={:.3f}, pres_tri_tick={:.3f}'.format(
                pres_val[0], pres_val[pres_tri_index], pres_val[-1], pres_tri_index, pres_tri_tick))
            self.print_msg('TRI_STEP_MSG', 'start_step={:.3f}, tri_step={:.3f}, end_step={:.3f}, step_tri_index={:.3f}, step_tri_tick={:.3f}'.format(
                start_step, tri_step, step_val[-1], step_tri_index, step_tri_tick))
            self.print_msg('TRI_OUT_MM', 'mm_pre_step={:.3f}, out_mm={:.3f} start_pos_z={:.3f} cal_tri_z={:.3f} oft_z={:.3f}'.format(
                self.mm_per_step, out_mm, start_pos_z, cal_tri_z, oft_z))
            out_mms.append(out_mm)
            self.trig_info[ch] = pres_tri_index
        self.print_res()
        self.print_ary('OUT_MMS', out_mms, len(out_mms), 2)
        return sum(out_mms) / len(out_mms)

    # lines 760-771. Signature from the kwarg table at 0x11f500.
    def get_step_cnts(self, run_dis, run_spd):
        if not self.mm_per_step:                                          # 761
            self.get_mm_per_step()                                        # 762
        step_cnt = int(run_dis / self.mm_per_step)                        # 763
        if step_cnt == 0:                                                 # 764
            self.print_msg('get_step_cnts', 'run_dis:%s mm_per_step:%s step_cnt:%s run_spd:%s' % (
                run_dis, self.mm_per_step, int(run_dis / self.mm_per_step), run_spd))  # 765-766
            return step_cnt, 0, 0                                         # 767
        step_us = int(((run_dis / run_spd) * 1000 * 1000) / step_cnt)     # 768
        acc_ctl_cnt = int(self.acc_ctl_mm / self.mm_per_step)             # 769
        self.print_msg('GET_STEP_CNTS', 'run_dis=%.2f, run_spd=%.2f, step_cnt=%d, step_us=%d acc_ctl_cnt=%d' % (
            run_dis, run_spd, step_cnt, step_us, acc_ctl_cnt))            # 770
        return step_cnt, step_us, acc_ctl_cnt                             # 771

    # get_best_rdy_z: original lines 773-783 (body FUN_000528a4).
    # Confidence: high; all 52 traced calls (G29 26, CBM 26) reproduce args of the three
    # get_linear2 calls, the print_msg text and the return value (recon/check_shake.py).
    # Same algorithm as the open-source prtouch.py get_best_rdy_z.
    def get_best_rdy_z(self, rdy_x, rdy_y, base_pos=None):
        if not base_pos:                                                            # 774
            base_pos = self.rdy_pos                                                 # 775
        p_left = [base_pos[0][0], rdy_y, 0]                                         # 776
        p_right = [base_pos[2][0], rdy_y, 0]                                        # 777
        p_mid = [rdy_x, rdy_y, 0]                                                   # 778
        p_left = self.get_linear2(base_pos[0], base_pos[1], p_left, False)          # 779
        p_right = self.get_linear2(base_pos[2], base_pos[3], p_right, False)        # 780
        p_mid = self.get_linear2(p_left, p_right, p_mid, True)                      # 781
        self.print_msg('GET_BEST_RDY_Z', 'Src=%s, x=%.2f, y=%.2f, cal_z=%.2f' % (
            ('RDY' if base_pos == self.rdy_pos else 'GAP'), rdy_x, rdy_y, p_mid[2]))  # 782
        return p_mid[2] if p_mid[2] < self.bed_max_err else self.bed_max_err       # 783

    # lines 785-791. No break: every Z stepper overwrites mm_per_step and logs.
    def get_mm_per_step(self):
        if self.mm_per_step:                                              # 786
            return
        for stepper in self.toolhead.get_kinematics().get_steppers():     # 788
            if stepper.is_active_axis('z'):                               # 789
                self.mm_per_step = self.step_base * stepper.get_step_dist()  # 790
                self.print_msg('GET_MM_PER_STEP', str(stepper.get_step_dist()))  # 791

    # ------------------------------------------------------------------
    # run_to_next  (py lines 794-802; wrapper M_run_to_next, body FUN_0005098c)
    # Called by probe.py _move_next as run_to_next(nextpos) with a 2-element
    # [x, y], and by run_G29_Z / run_re_g29s as run_to_next(pos, True).
    # Confidence: high; both branches are traced (trace_G29.log:3 and :9857).
    # ------------------------------------------------------------------
    def run_to_next(self, nextpos, wait=True):
        if self.After_G28Z_skip_bed_tilt_flag == False:                          # 795
            self.print_msg('RUN_TO_NEXT', 'nextpos=%s, wait=%d' % (str(nextpos), wait))  # 796
            cal_nextpos_z = self.get_best_rdy_z(nextpos[0], nextpos[1]) + self.best_above_z  # 797
        else:
            cal_nextpos_z = self.bed_max_err                                       # 800
            self.print_msg('RUN_TO_NEXT',                                          # 801
                           'After G28Z first run fun(run_to_next) skip bed tilt flag:{} nextpos:{} cal_nextpos_z:{}'.format(
                               self.After_G28Z_skip_bed_tilt_flag, nextpos, cal_nextpos_z))
        self.move(nextpos[:2] + [cal_nextpos_z], self.rdy_xy_spd, wait)           # 802

    # ------------------------------------------------------------------
    # env_self_check  (py lines ~812-1092; wrapper M_env_self_check, body
    # FUN_000f5468 reached through the FUN_000f5458 thunk)
    # Signature from the argname array and wrapper defaults: error_num=1,
    # force=False.
    # Called by run_G28_Z and cmd_SELF_CHECK_PRTOUCH as `self.env_self_check()`; a falsy
    # return makes run_G28_Z print 'FUN:G28_Z env_self_check() ERROR'.
    #
    # Structure:
    #   up to 10 attempts of the static tests 1-4 (any failure retries; after
    #   10 failed attempts the last failure code picks the raised error, a
    #   swap-pin failure raises nothing and just returns False), then up to 2
    #   attempts of the shake test 5, then (1..3 channels unsensed) up to 2
    #   attempts of a probe-based re-check (lines 1010-1086).
    # Confidence: high for every statement on the traced path (trace_G28.log
    # 124-756, trace_AHZ.log 101-735). Failure branches are decompile-only;
    # the code dispatch after the 10-attempt loop was confirmed in the MIPS
    # disassembly at file VA 0xe5508 (only 3, 4 and 2 are tested).
    # The 1010-1086 fallback is untraced; it was read statement by statement
    # from the disassembly (per-statement confidence in selfcheck.notes.md)
    # and is exercised by the synthetic scenarios in check_selfcheck.py.
    # ------------------------------------------------------------------
    def env_self_check(self, error_num=1, force=False):
        err = 0  # C-int local in the original (Ghidra in_v1); name unknown
        for i in range(10):
            self.delay_s(0.2)                                                    # 815
            self.print_msg('WHY_DEBUG', 'Static Check Sensor data! times:{}'.format(i))  # 816

            # Self test 1: sync (swap) pin.
            self.write_swap_prtouch_cmd.send([self.pres_oid, 1])                 # 817
            params1 = self.read_swap_prtouch_cmd.send([self.step_oid])           # 818
            self.write_swap_prtouch_cmd.send([self.pres_oid, 0])                 # 819
            params0 = self.read_swap_prtouch_cmd.send([self.step_oid])           # 820
            if not params0 or not params1 or params0['sta'] != 0 or params1['sta'] != 1:  # 821
                self.print_msg('DEBUG', '--Self Test 1 = PR_ERR_CODE_SWAP_PIN_DETECTI, Error!! params0:{} params1:{}--'.format(params0, params1), force)  # 822
                err = 1
                continue
            self.print_msg('DEBUG', '--Self Test 1 = PR_ERR_CODE_SWAP_PIN_DETECTI, Pass!!--', force)  # 827

            if (not self.need_self_check and not force) or self.use_adc:          # 834
                return True

            # Self test 2: read interval.
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])        # 841
            self.print_msg('AVGS_RESAULT', str(params))                          # 842
            self.pres_res = []                                                   # 843
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 40])  # 844
            t_start = time.time()                                                # 845
            while (time.time() - t_start) < 1.5 * (self.tri_acq_ms / 1000.0) * 64 and len(self.pres_res) < 32:  # 847
                self.delay_s(0.01)                                               # 848
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 0])  # 850
            if len(self.pres_res) < 32:                                          # 852
                err = 2
                continue
            ticks, datas = [], [[], [], [], []]                                  # 857
            for j in range(4, len(self.pres_res) - 4):                           # 859
                ticks.append(self.pres_res[j]['tick'] / 10000.0)                 # 860
                for k in range(self.pres_cnt):                                   # 861
                    datas[k].append(self.pres_res[j]['ch%d' % k])                # 862
            tick_sum = 0
            for j in range(1, len(ticks)):                                       # 864
                tick_sum += ticks[j] - ticks[j - 1]                              # 865
            self.print_msg('SELF_CHECK_TICK', 'TICK:{}'.format(ticks))           # 867
            self.print_msg('SELF_CHECK_DATA', 'DATA:{}'.format(datas))           # 868
            for j in range(len(datas)):                                          # 870
                self.send_wave_tri(j, datas[j])                                  # 871
            if tick_sum / (len(ticks) - 1) > 2 * self.tri_acq_ms:                # 873
                self.print_msg('DEBUG', '--Self Test 2 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Error!!-- error time:{} Normal:{}'.format(tick_sum / (len(ticks) - 1), 2 * self.tri_acq_ms), force)  # 875
                err = 2
                continue
            self.print_msg('DEBUG', '--Self Test 2 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Pass!!--', force)  # 879

            # Self test 3: constant value (longest run of equal neighbours).
            IS_CONSTANT_CH = []                                                  # 884
            for j in range(self.pres_cnt):                                       # 885
                cnt = 0                                                          # 887
                for k in range(1, len(datas[j])):                                # 888
                    if datas[j][k] == datas[j][k - 1]:                           # 891
                        cnt += 1                                                 # 892
                    else:
                        cnt = 0
                if cnt > len(datas[j]) // 2:                                     # 896
                    IS_CONSTANT_CH.append(j)                                     # 897
            if len(IS_CONSTANT_CH) >= error_num:                                 # 898
                self.print_msg('DEBUG', '--Self Test 3 = PR_ERR_CODE_PRES_VAL_IS_CONSTANT, Error!!-- IS_CONSTANT_CH:{}'.format(IS_CONSTANT_CH), force)  # 900
                err = 3
                continue
            self.print_msg('DEBUG', '--Self Test 3 = PR_ERR_CODE_PRES_VAL_IS_CONSTANT, Pass!!--', force)  # 904

            # Self test 4: noise.
            TOO_BIG_CH = []                                                      # 910
            for j in range(self.pres_cnt):                                       # 911
                cnt = 0                                                          # 912
                for k in range(len(datas[j])):                                   # 913
                    cnt += 1 if abs(datas[j][k]) > (self.tri_min_hold[0] * 2 if not self.use_adc else 200) else 0  # 914
                if cnt > len(datas[j]) // 2:                                     # 915
                    TOO_BIG_CH.append(j)                                         # 916
            if len(TOO_BIG_CH) >= error_num:                                     # 917
                self.print_msg('DEBUG', '--Self Test 4 = PR_ERR_CODE_PRES_NOISE_TOO_BIG, Error!!-- TOO_BIG_CH:{}'.format(TOO_BIG_CH), force=force)  # 919
                err = 4
                continue
            self.print_msg('DEBUG', '--Self Test 4 = PR_ERR_CODE_PRES_NOISE_TOO_BIG, Pass!!--', force=force)  # 923
            break
        else:
            if err == 2:
                self.ck_and_raise_error(True, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT)  # 928
            elif err == 3:
                self.ck_and_raise_error(True, PR_ERR_CODE_PRES_VAL_IS_CONSTANT, IS_CONSTANT_CH)  # 930
            elif err == 4:
                self.ck_and_raise_error(True, PR_ERR_CODE_PRES_NOISE_TOO_BIG, TOO_BIG_CH)  # 932
            return False

        # Self test 5: shake the Z axis and require each channel to see it.
        # t_start is NOT reset here: the wait loop at 954 measures from the
        # last self-test-2 start (faithful to the original).
        for j in range(2):
            self.delay_s(0.2)                                                    # 938
            self.print_msg('WHY_DEBUG', 'Shake Check Sensor data! times:{}'.format(j))  # 939
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])                 # 941
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])         # 942
            self.print_msg('AVGS_RESAULT', str(params))                          # 943
            now_pos = self.toolhead.get_position()                               # 944
            self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])  # 945
            self.pres_res = []                                                   # 946
            self.prtouch_shake_check_error = False                               # 947
            sample_num = self.shake_sample_num                                   # 949
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, sample_num + 25])  # 951
            self.quick_shake_motor(self.shake_num)                               # 953
            while (time.time() - t_start) < 1.5 * (self.tri_acq_ms / 1000.0) * sample_num and len(self.pres_res) < sample_num:  # 954
                self.delay_s(0.01)                                               # 955
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 0])  # 956
            if len(self.pres_res) < 32:                                          # 957
                self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Error!!-- read {} pres data len {} < 32'.format(sample_num, len(self.pres_res)), force)  # 958
                if j != 0:
                    self.ck_and_raise_error(True, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT)  # 960
                    return False
                continue
            ticks, datas = [], [[], [], [], []]                                  # 964
            for k in range(25, len(self.pres_res)):                              # 966
                ticks.append(self.pres_res[k]['tick'] / 10000.0)                 # 967
                for m in range(self.pres_cnt):                                   # 968
                    datas[m].append(self.pres_res[k]['ch%d' % m])                # 969
            for k in range(len(datas)):                                          # 971
                self.send_wave_tri(k, datas[k])                                  # 972
                self.print_msg('SELF_CHECK_DATA', 'SHAKE DATA CH[{}]:{}'.format(k, datas[k]))  # 973
            NO_SENSED_CH = []
            for k in range(self.pres_cnt):
                datas[k].sort()
                datas[k] = datas[k][3:len(datas[k]) - 3]
                n = len(datas[k])
                avg = sum(datas[k]) / n
                pres_std = 0
                for m in range(n):
                    pres_std += math.pow(datas[k][m] - avg, 2)                   # 993
                pres_std = math.sqrt(pres_std / n)                               # 995
                if pres_std < self.check_std_mim:                                # 997
                    NO_SENSED_CH.append(k)                                       # 998
                self.print_msg('DEBUG', ' pres_std[{}]:{:.2f} normal:[{}]'.format(k, pres_std, self.check_std_mim))  # 1000
            if len(NO_SENSED_CH) == 0:                                           # 1002
                break
            self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_NOT_BE_SENSED, Error!!-- Error CH:{}'.format(NO_SENSED_CH), force)  # 1004-1005

        # Lines 1010-1086: probe-based re-check, only when 1..3 channels were
        # not sensed (MIPS: unsigned `len-1 < 3` at file VA 0xf1400; with all 4
        # channels unsensed it jumps straight to the 1088 check). Recovered
        # from the disassembly of FUN_000f5468 (file VA 0xf13e4-0xf8584).
        if 0 < len(NO_SENSED_CH) < 4:                                        # 1010
            z_down_spd = self.tri_z_down_spd[0]                              # 1012
            hftr_cut = self.tri_hftr_cut[0]                                  # 1013
            lftr_k1 = self.tri_lftr_k1[0]                                    # 1014
            min_hold = self.tri_min_hold[0]                                  # 1015
            max_hold = self.tri_max_hold[0]                                  # 1016
            # Two attempts: a C down-counter (2 -> 1) in the binary. Every
            # `continue` below goes to the counter test at 0xf3860; after the
            # second attempt control falls to the 1088 check.
            for _ in range(2):
                self.enable_steps()                                          # 1019
                self.print_msg('WHY DEBUG', 'check sensor sensitivity')      # 1020
                self.step_res, self.pres_res = [], []                        # 1021
                params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])  # 1022
                self.print_msg('AVGS_RESAULT', str(params))                  # 1023
                step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(
                    self.max_z * 1.2, z_down_spd * (1.2 if self.use_adc else 2.0))  # 1024
                self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms,
                                                  self.tri_need_cnt, int(hftr_cut * 1000), int(lftr_k1 * 1000),
                                                  int(min_hold), int(max_hold)])  # 1025-1026
                self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt, step_us,
                                                  acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 1027
                t_start = time.time()                                        # 1028
                while (time.time() - t_start) < (self.max_z * 2.0) / (z_down_spd * (1.2 if self.use_adc else 2.0)) \
                        and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):  # 1029
                    self.delay_s(0.01)                                       # 1030
                self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0,
                                                  self.low_spd_nul, self.send_step_duty, 0])  # 1031
                self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])  # 1032
                if len(self.pres_res) == 0 or len(self.step_res) == 0:       # 1035
                    self.print_msg('WHY DEBUG', '--Self Test 5 bed move max_z*1.2 is not probe,move again--')  # 1036
                    continue
                self.step_res = []                                           # 1040
                up_dis = self.best_above_z * 2                               # 1041
                # The up move uses the DOWN speed (z_down_spd), as in the binary.
                step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(
                    up_dis, z_down_spd * (1.2 if self.use_adc else 2.0))     # 1042
                self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt, step_us,
                                                  acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 1043
                t_start = time.time()                                        # 1044
                while (time.time() - t_start) < up_dis / z_down_spd + 5 and len(self.step_res) != MAX_BUF_LEN:  # 1045
                    self.delay_s(0.01)                                       # 1046
                # Stop is sent with dir 0 (not 1), and no start_pres stop here.
                self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0,
                                                  self.low_spd_nul, self.send_step_duty, 0])  # 1047
                if len(self.pres_res) != MAX_BUF_LEN:                        # 1050
                    self.print_msg('WHY DEBUG', '--Self Test 5 pres data len {} < 32--'.format(len(self.pres_res)))  # 1051
                    continue
                datas = [[], [], [], []]                                     # 1055
                NO_SENSED_CH = []                                            # 1056
                for i in range(self.pres_cnt):                               # 1057
                    for j in range(len(self.pres_res)):                      # 1058
                        datas[i].append(self.pres_res[j]['ch%d' % i])        # 1059
                # High-pass filter. The binary holds the C double 0.5 and divides
                # by math.pi, then by self.cal_hftr_cut (not the tri_hftr_cut local).
                rc = 0.5 / math.pi / self.cal_hftr_cut                       # 1062
                alpha = rc / (rc + 1.0 / (1000 / self.tri_acq_ms))           # 1063
                NO_SENSED_CH = []                                            # 1064
                for i in range(len(datas)):                                  # 1065
                    pnt_vals = [0]                                           # 1066
                    for j in range(1, len(datas[i])):                        # 1067
                        pnt_vals.append(round((datas[i][j] - datas[i][j - 1] + pnt_vals[-1]) * alpha, 2))  # 1068
                    datas[i] = pnt_vals                                      # 1069
                    if datas[i][-1] < self.check_std_mim:                    # 1071
                        NO_SENSED_CH.append(i)                               # 1072
                    # The message says [-1] but the binary formats the whole list.
                    self.print_msg('DEBUG', ' pnt_vals[{}][-1]:{} normal:[{}]'.format(
                        i, datas[i], self.check_std_mim))                    # 1073
                if len(NO_SENSED_CH) >= error_num:                           # 1075
                    self.prtouch_shake_check_error = True                    # 1076
                    continue
                self.prtouch_shake_check_error = False                       # 1079
                for i in range(len(datas)):                                  # 1083
                    self.send_wave_tri(i, datas[i])                          # 1084
                break
        if len(NO_SENSED_CH) and self.prtouch_shake_check_error == True:        # 1088
            self.ck_and_raise_error(True, PR_ERR_CODE_PRES_NOT_BE_SENSED, NO_SENSED_CH)  # 1089
            return False
        self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_NOT_BE_SENSED, Pass!!--', force)  # 1091
        return True

    # ------------------------------------------------------------------
    # probe_ready  (py lines 1094-1112; M_probe_ready, METH_O, body inline)
    # Probes the four rdy_pos corners (set up in _build_step_config from the
    # bed mesh min/max) and stores each trigger Z in rdy_pos[i][2].
    # Returns True, or False when jump_probe_ready was set or a probe failed.
    # Confidence: high (trace_G29.log:104-11977). The False returns are
    # confirmed from the disassembly (Py_False loaded before the return).
    # ------------------------------------------------------------------
    def probe_ready(self):
        self.print_msg('PROBE_READY', 'Start probe_ready()...')                   # 1095
        self.After_G28Z_skip_bed_tilt_flag = False                                # 1096
        if self.jump_probe_ready:                                                 # 1097
            self.jump_probe_ready = False                                         # 1098
            return False                                                          # 1099
        self.ck_g28ed()                                                           # 1100
        for i in range(len(self.rdy_pos)):                                        # 1101
            self.rdy_pos[i][2] = self.bed_max_err                                 # 1102
        now_pos = self.toolhead.get_position()                                    # 1103
        self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.rdy_z_spd)   # 1104
        for i in range(4):                                                        # 1105
            self.print_msg('PROBE_READY', 'Start Probe Point=%s' % str(self.rdy_pos[i]))  # 1106
            self.move(self.rdy_pos[i], self.rdy_xy_spd)                           # 1107
            self.rdy_pos[i][2], self.run_step_prtouch_flag = self.run_step_prtouch(  # 1108
                self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            if not self.run_step_prtouch_flag:                                    # 1109
                self.print_msg('WHY ERROR', 'FUN:probe_ready run_step_prtouch() ERROR')  # 1110
                return False                                                      # 1111
        self.print_msg('RDY_POS', '[00=%.2f, 01=%.2f, 11=%.2f, 10=%.2f]' % (       # 1112
            self.rdy_pos[0][2], self.rdy_pos[1][2], self.rdy_pos[2][2], self.rdy_pos[3][2]))
        return True

    # line 1115-1116. Not traced.
    def report_run_dis(self, run_dis):
        self.v_sd.run_dis = run_dis

    # safe_move_z: original lines 1118-1146 (FUN_0004ad0c, wrapper and body inlined).
    # Signature from the argname array: (self, run_sta, run_dis, run_spd, run_rdo,
    # tri_call_back=None); default None from the M_safe_move_z wrapper.
    # UNTESTED: no trace exercises SAFE_MOVE_Z. Reconstructed from the decompile, with the
    # send-list contents and the use_adc conditionals confirmed in the MIPS disassembly
    # (recon/_work_shake/safe_dis.txt).
    # The direction is computed as a C long (PyLong_FromLong of (run_dis > 0)), so the source
    # was most likely `1 if run_dis > 0 else 0`; the local name is unknown ("run_dir" here).
    # After the stop commands, run_sta != 1 returns None; so do zero speed/distance/ratio and
    # a zero entry in the get_step_cnts result.
    def safe_move_z(self, run_sta, run_dis, run_spd, run_rdo, tri_call_back=None):
        self.print_msg('SAFE_MOVE_Z', 'run_sta=%d, run_dis=%f, run_spd=%f, run_rdo=%f' % (
            run_sta, run_dis, run_spd, run_rdo))                                    # 1119
        run_dir = 1 if run_dis > 0 else 0                                           # 1120
        run_dis = math.fabs(run_dis)                                                # 1121
        if tri_call_back:                                                           # 1123
            self.safe_move_z_tri_call_back = tri_call_back                          # 1124
        else:
            self.safe_move_z_tri_call_back = self.report_run_dis                    # 1126
        self.start_step_prtouch_cmd.send([self.step_oid, run_dir, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])  # 1128
        self.start_pres_prtouch_cmd.send([self.pres_oid, run_dir, 0, 0, 0, 0, 0, 0, 0])  # 1129
        if run_sta == 1:                                                            # 1131
            self.step_res, self.pres_res = [], []                                   # 1132
            self.enable_steps()                                                     # 1133
            if run_spd == 0 or run_dis == 0 or run_rdo == 0:                        # 1134
                return
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])                     # 1137
            step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(run_dis, run_spd)   # 1138
            if step_cnt == 0 or step_us == 0 or acc_ctl_cnt == 0:                   # 1140
                return
            self.safe_move_z_all_cnt = step_cnt                                     # 1142
            self.start_pres_prtouch_cmd.send([self.pres_oid, run_dir, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt,
                                              int(self.tri_hftr_cut[0] * 1000),
                                              int(self.tri_lftr_k1[0] * 1000) if self.use_adc else int(self.tri_lftr_k1[0] * 1000 / run_rdo),
                                              self.tri_min_hold[0] if self.use_adc else int(self.tri_min_hold[0] * run_rdo),
                                              self.tri_max_hold[0] if self.use_adc else int(self.tri_max_hold[0] * run_rdo)])  # 1143-1145
            self.start_step_prtouch_cmd.send([self.step_oid, run_dir, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt,
                                              self.low_spd_nul, self.send_step_duty, 0])  # 1146

    # run_step_prtouch: original lines 1156-1306 (body FUN_0008d328).
    # rt_last is accepted but never used by the body (the arg wrapper does not pass it on).
    # Local variable names are invented (Cython keeps no names for C locals); everything
    # else (attribute names, constants, operand order, strings) is from the binary.
    # Confidence: high for the traced path (down move, cal_tri_data, partial/total up move,
    # result selection); medium for the untraced branches (no-trigger, re_g28, refresh_z,
    # threshold blending), whose structure is decoded but never exercised by a trace.
    def run_step_prtouch(self, down_min_z, probe_min_3err, rt_last=False, pro_cnt=3, crt_cnt=3, fast_probe=False, re_g28=False):
        tri_z_down_spd = self.tri_z_down_spd[0]
        tri_hftr_cut = self.tri_hftr_cut[0]
        tri_lftr_k1 = self.tri_lftr_k1[0]
        tri_min_hold = self.tri_min_hold[0]
        tri_max_hold = self.tri_max_hold[0]

        self.print_msg('RUN_STEP_PRTOUCH', 'tri_z_down_spd=%.2f, tri_hftr_cut=%.2f, tri_lftr_k1=%.2f, tri_min_hold=%.0f, tri_max_hold=%.0f' % (
            tri_z_down_spd, tri_hftr_cut, tri_lftr_k1, tri_min_hold, tri_max_hold))

        res_z = []
        now_pos = self.toolhead.get_position()
        cnt, step_err, pres_err = 0, 0, 0
        no_tri_cnt, oft_steps = 0, 0
        # C-typed (bint) flag in the binary: set when the no-trigger path re-zeroed Z, so the
        # next real trigger re-homes Z at the contact point (tested as `== 1` at L1240).
        refresh_z = False
        while True:
            self.step_res, self.pres_res = [], []
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(down_min_z, tri_z_down_spd)
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt,
                                              int(tri_hftr_cut * 1000), int(tri_lftr_k1 * 1000), int(tri_min_hold), int(tri_max_hold)])
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t = time.time()
            while time.time() - t < down_min_z / tri_z_down_spd + 2 and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):
                self.delay_s(0.01)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])

            # L1183-1207: nothing came back at all (no trigger over the whole down_min_z travel).
            if len(self.step_res) == 0 or len(self.pres_res) == 0:
                if re_g28:
                    self.move(now_pos[:2] + [100, now_pos[3]], self.rdy_z_spd)
                    self.run_G28_Z(True)
                    self.probe_ready()
                    self.move(now_pos, self.rdy_z_spd)
                else:
                    no_tri_cnt += 1
                    if self.z_set_status == True:
                        refresh_z = True
                        self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
                        self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.tri_z_up_spd)
                        self.toolhead.wait_moves()
                        now_pos = self.toolhead.get_position()
                        self.print_msg('WHY DEBUG', 'Prtouch Z Move {}mm but loadcell No Trigger times:{},reset z level and prtouch z again!'.format(down_min_z, no_tri_cnt))
                    else:
                        self.print_msg('WHY DEBUG', 'Prtouch Z Move {}mm but loadcell No Trigger times:{}'.format(down_min_z, no_tri_cnt))
                        self.ck_and_raise_error(True, PR_ERR_CODE_PR_NOT_TRIGGER, ['Z Move {}mm but loadcell No Trigger times:{}'.format(down_min_z, no_tri_cnt)])
                continue

            # L1209-1217
            if not self.ck_and_manual_get_step():
                step_err += 1
                self.print_msg('WHY ERROR', 'FUN:run_step_prtouch ck_and_manual_get_step() ERROR Times:{}', step_err)
                if step_err > 2:
                    self.ck_and_raise_error(True, PR_ERR_CODE_STEP_LOST_RUN_DATA, ['step_res len:{}'.format(len(self.step_res))])
                    return 999, False
            else:
                step_err = 0

            # L1219-1227
            if not self.ck_and_manual_get_pres():
                pres_err += 1
                self.print_msg('WHY ERROR', 'FUN:run_step_prtouch ck_and_manual_get_pres() ERROR Times:{}', pres_err)
                if pres_err > 2:
                    self.ck_and_raise_error(True, PR_ERR_CODE_PRES_LOST_RUN_DATA, ['pres_res len:{}'.format(len(self.pres_res))])
                    return 999, False
            else:
                pres_err = 0

            # L1233-1251: re-home Z at the contact point the probe is resting on right now
            # (no up move was made yet), then restart the whole measurement.
            if self.nozzle_clear_z_out_of_range == True:
                self.print_msg('WHY DEBUG', 'nozzle clear z less than z_min:{},reset z level and prtouch z again!'.format(self.min_z))
                self.nozzle_clear_z_out_of_range = False
                refresh_z = True
            if refresh_z == True:
                self.print_msg('WHY DEBUG', 'refresh z home position now_pos{}'.format(now_pos))
                self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
                self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.tri_z_up_spd)
                self.toolhead.wait_moves()
                now_pos = self.toolhead.get_position()
                res_z.clear()
                cnt = 0
                refresh_z = False
                continue

            can_rt = False
            if len(self.pres_res) >= MAX_BUF_LEN or cnt > 20:
                step_res, pres_res, tri_time = [s for s in self.step_res], [p for p in self.pres_res], [self.pres_tri_time, self.step_tri_time]
                res_z.append(self.cal_tri_data(step_cnt, now_pos[2], step_res, pres_res, -oft_steps * self.mm_per_step))
                can_rt = (len(res_z) == 2 and not self.use_adc and math.fabs(res_z[0] - res_z[1]) < 0.05 and crt_cnt != pro_cnt) or \
                         (len(res_z) >= crt_cnt - 1 and max(res_z) - min(res_z) <= probe_min_3err)
            else:
                # L1262-1270: the pressure buffer did not fill; blend the trigger parameters from
                # their [0] towards their [1] value over retries 5..20, shake every 5th retry.
                if cnt <= 20 and cnt >= 5:
                    k = (cnt - 5) / 15.0
                    tri_z_down_spd = (self.tri_z_down_spd[1] - self.tri_z_down_spd[0]) * k + self.tri_z_down_spd[0]
                    tri_hftr_cut = (self.tri_hftr_cut[1] - self.tri_hftr_cut[0]) * k + self.tri_hftr_cut[0]
                    tri_lftr_k1 = (self.tri_lftr_k1[1] - self.tri_lftr_k1[0]) * k + self.tri_lftr_k1[0]
                    tri_min_hold = (self.tri_min_hold[1] - self.tri_min_hold[0]) * k + self.tri_min_hold[0]
                    tri_max_hold = (self.tri_max_hold[1] - self.tri_max_hold[0]) * k + self.tri_max_hold[0]
                if cnt % 5 == 0:
                    self.shake_motor(self.shake_cnt)

            # L1272-1283: up move. With fast_probe, after the first sample only lift to
            # best_above_z/2 and remember the skipped steps (oft_steps); later samples start
            # from there (cal_tri_data gets oft_z = -oft_steps*mm_per_step) and the last up move
            # adds the skipped steps back so the head returns to its start height.
            up_min_z = (step_cnt - self.step_res[-1]['step']) * self.mm_per_step
            self.print_msg('WHY DEBUG', 'run_step_prtouch up_min_z:{} can_rt:{}'.format(up_min_z, can_rt))
            up_step_cnt, up_step_us, up_acc_ctl_cnt = self.get_step_cnts(up_min_z, self.tri_z_up_spd)
            up_steps = int(step_cnt - self.step_res[-1]['step'])
            if fast_probe and pro_cnt > 2 and len(res_z) > 0:
                if len(res_z) == 1 and up_min_z > self.best_above_z / 2:
                    up_step = int(self.best_above_z / 2 / self.mm_per_step)
                    oft_steps = up_steps - up_step
                    up_steps = int(up_step)
                elif len(res_z) >= pro_cnt or can_rt:
                    up_steps += int(oft_steps)

            self.step_res, self.pres_res = [], []
            self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, up_steps, up_step_us, up_acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t = time.time()
            while time.time() - t < down_min_z / tri_z_down_spd + 5 and len(self.step_res) != MAX_BUF_LEN:
                self.delay_s(0.01)
            self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])

            # L1293-1295
            if len(res_z) >= pro_cnt or can_rt:
                break
            cnt += 1

        # L1300-1306
        self.print_ary('RES_Z', res_z, len(res_z))
        self.Tri_bed_test_z = [round(z, 3) for z in res_z]
        res_z.sort()
        res = (res_z[0] + res_z[1]) / 2 if len(res_z) == 2 else res_z[int((len(res_z) - 1) // 2)]
        self.print_msg('WHY DEBUG', 'OUT_RES_Z:{:.3f}'.format(res))
        return res, True

    # ------------------------------------------------------------------
    # bed_mesh_post_proc  (py lines 1308-1393; wrapper FUN_000a99f4,
    # body FUN_0009ff2c)
    # Called by run_G29_Z on the last mesh point with now_pos (4 elements).
    # Appends last_point to bed_mesh.bmc.probe_helper.results, re-probes up to
    # 3 rounds of "spike" points (slope to a 4-neighbour > 3 / mesh span),
    # adds noz_ex_com and the linear-in-Y tilt correction to every Z, lifts
    # the whole mesh so the centre Z is >= noz_ex_com, then removes the
    # appended last_point again (del results[-1]). All edits are in place:
    # last_point itself gets the same correction, which is why run_G29_Z
    # returns the corrected Z for the last point.
    # Confidence: high for 1309-1318, 1365-1367 and 1387-1393 (traced, values
    # bit-exact). The spike detection/re-probe loop (1321-1362) and the
    # centre-Z cases other than odd x odd (1374-1385) are decompile-only.
    # ------------------------------------------------------------------
    def bed_mesh_post_proc(self, last_point):
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']                      # 1309
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']                      # 1310
        min_x, min_y = self.bed_mesh.bmc.mesh_min                             # 1311
        max_x, max_y = self.bed_mesh.bmc.mesh_max                             # 1312
        res = self.bed_mesh.bmc.probe_helper.results                          # 1314
        res.append(last_point)                                                # 1315
        self.print_msg('WHY DEBUG', 'bed mesh last point=:{}'.format(last_point))  # 1317
        self.print_msg('BED_MESH_POST_PROC', 'Before=' + str(res))           # 1318
        # Outer loop is a C loop (counter 3 counting down, index unused).
        for _ in range(3):
            # Rows in probe (serpentine) order -> rows in X order.
            points = []                                                       # 1322
            for i in range(y_cnt):                                            # 1323
                points.append(res[int(i * x_cnt):int(i * x_cnt + x_cnt)][::(1 if i % 2 == 0 else -1)])  # 1324
            err_ps = []                                                       # 1326
            err_idx = []                                                      # 1327
            max_tan = 3 / ((max_y - min_y) if (max_y - min_y) < (max_x - min_x) else (max_x - min_x))  # 1329
            for i in range(len(points)):                                      # 1330
                for j in range(len(points[i])):                               # 1332
                    p = points[i][j]                                          # 1333
                    # NB: the row bound uses x_cnt and the column bound uses
                    # y_cnt (swapped vs. the loops); faithful to the binary.
                    nbs = [points[i - 1][j] if i >= 1 else p,                 # 1334
                           points[i + 1][j] if i < x_cnt - 1 else p,
                           points[i][j - 1] if j >= 1 else p,                 # 1335
                           points[i][j + 1] if j < y_cnt - 1 else p]
                    cnt = 0                                                   # 1337
                    for k in range(len(nbs)):                                 # 1338
                        if nbs[k] != p:
                            dz = math.fabs(nbs[k][2] - p[2])
                            dy = math.fabs(nbs[k][1] - p[1])
                            dx = math.fabs(nbs[k][0] - p[0])
                            if dz / (dy if dy > dx else dx) > max_tan:
                                cnt += 1                                      # 1339
                    if cnt >= 1 and p not in err_ps:                          # 1340
                        err_ps.append(p)                                      # 1341
                        err_idx.append([i, j])                                # 1343
            self.print_msg('RE_PROBE_POINT', 'max_tan:{} len:{} err_ps:{}'.format(max_tan, len(err_ps), err_ps))  # 1345
            for i in range(len(err_ps)):                                      # 1346
                self.run_to_next(err_ps[i])                                   # 1347
                err_ps[i][2], self.run_step_prtouch_flag = self.run_step_prtouch(  # 1348
                    self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
                if not self.run_step_prtouch_flag:                            # 1349
                    self.print_msg('WHY ERROR', 'FUN:bed_mesh_post_proc run_step_prtouch() ERROR')  # 1350
                    return False                                              # 1351
            for i in range(len(err_ps)):                                      # 1353
                if err_idx[i][0] % 2 == 0:                                    # 1356
                    idx = err_idx[i][1] + err_idx[i][0] * x_cnt               # 1357
                else:
                    idx = err_idx[i][0] * x_cnt + x_cnt - err_idx[i][1] - 1   # 1360
                self.print_msg('RE_PROBE_POINT', 'new_res1[{}]:{}  old_res1{}:{}'.format(
                    idx, err_ps[i], idx, res[idx]))                           # 1361
                res[idx] = err_ps[i]                                          # 1362
        for i in range(len(res)):                                             # 1365
            res[i][2] += 0 + self.noz_ex_com                                  # 1366
            res[i][2] += (1 - (res[i][1] - min_y) / (max_y - min_y)) * (2 * self.tilt_corr_dis) - self.tilt_corr_dis  # 1367
        center_z = self.noz_ex_com                                            # 1370
        if x_cnt % 2 == 1 and y_cnt % 2 == 1:                                 # 1371
            center_z = res[int(len(res) / 2)][2]                              # 1372
        else:
            if x_cnt % 2 == 0 and y_cnt % 2 == 0:                             # 1374
                axi = [[x_cnt / 2 - 1, y_cnt / 2 - 1], [x_cnt / 2, y_cnt / 2 - 1],
                       [x_cnt / 2 - 1, y_cnt / 2], [x_cnt / 2, y_cnt / 2]]    # 1375
            elif x_cnt % 2 == 1:                                              # 1376
                axi = [[int(x_cnt / 2), y_cnt / 2 - 1], [int(x_cnt / 2), y_cnt / 2]]  # 1377
            elif y_cnt % 2 == 1:                                              # 1378
                axi = [[x_cnt / 2 - 1, int(y_cnt / 2)], [x_cnt / 2, int(y_cnt / 2)]]  # 1379
            center_z = 0
            for i in range(len(axi)):                                         # 1383
                center_z += res[int(axi[i][0] + axi[i][1] * x_cnt)][2]        # 1384
            center_z /= len(axi)                                              # 1385
        if center_z < self.noz_ex_com:                                        # 1387
            diff = self.noz_ex_com - center_z                                 # 1388
            for i in range(len(res)):                                         # 1389
                res[i][2] += diff                                             # 1390
        self.print_msg('BED_MESH_POST_PROC', 'After=' + str(res))            # 1392
        del res[-1]                                                           # 1393
        return True

    # ------------------------------------------------------------------
    # check_bed_mesh  (py lines 1396-1437; wrapper M_check_bed_mesh,
    # body FUN_000a9bf8)
    # Called by cmd_CHECK_BED_MESH as check_bed_mesh(AUTO_G29 > 0).
    # auto_g29 is only printed: the re-level path runs regardless.
    # Probes the 4 rdy_pos corners (probe_ready), compares each with the
    # stored mesh minus noz_ex_com and the tilt correction. If fewer than 2
    # points exceed check_bed_mesh_max_err and [prtouch default] version
    # equals PR_VERSION it passes (returns True). Otherwise (or when there is
    # no mesh) it runs BED_MESH_CALIBRATE (jump_probe_ready=True makes that
    # G29 skip its own probe_ready), sets [prtouch default] version =
    # PR_VERSION in configfile and runs CXSAVE_CONFIG; returns True.
    # Returns False when probe_ready fails.
    # Confidence: high (trace_CBM.log, re-level branch). The pass branch
    # (1426-1427) and the no-mesh branch are decompile-only.
    # ------------------------------------------------------------------
    def check_bed_mesh(self, auto_g29=True):
        self.print_msg('CK_BED_MESH', 'Start check_bed_mesh({})...'.format(auto_g29))  # 1397
        self.ck_g28ed()                                                       # 1398
        min_x, min_y = self.bed_mesh.bmc.mesh_min                             # 1399
        max_x, max_y = self.bed_mesh.bmc.mesh_max                             # 1400
        mesh = self.bed_mesh.get_mesh()                                       # 1401
        self.bed_mesh.set_mesh(None)                                          # 1402
        self.print_msg('CK_BED_MESH', 'Start probe_ready()...')              # 1403
        self.probe_ready()                                                    # 1404
        if not self.run_step_prtouch_flag:                                    # 1405
            self.print_msg('WHY ERROR', 'FUN:check_bed_mesh probe_ready() ERROR')  # 1406
            return False                                                      # 1407
        self.jump_probe_ready = True                                          # 1408
        if self.use_adc:                                                      # 1409
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)           # 1410
        self.set_fan_speed('fan', 0.0)                                        # 1411
        if mesh:                                                              # 1412
            self.bed_mesh.set_mesh(mesh)                                      # 1413
            errs = []                                                         # 1414
            err_cnt = 0
            # C loop over 4 corners (bound is the constant 4).
            for i in range(4):
                mesh_z = self.bed_mesh.z_mesh.calc_z(self.rdy_pos[i][0], self.rdy_pos[i][1]) - self.noz_ex_com \
                    - ((1 - (self.rdy_pos[i][1] - min_y) / (max_y - min_y)) * (2 * self.tilt_corr_dis) - self.tilt_corr_dis)  # 1417
                errs.append(abs(self.rdy_pos[i][2] - mesh_z))                 # 1418
                err_cnt += 1 if errs[i] > self.check_bed_mesh_max_err else 0  # 1419
                self.print_msg('CK_BED_MESH', 'P%d = [x=%.2f, y=%.2f, mest_z=%.2f, probe_z=%.2f, err_z=%.2f]' % (
                    i, self.rdy_pos[i][0], self.rdy_pos[i][1], mesh_z, self.rdy_pos[i][2], errs[i]))  # 1420
            cfg_version = self.stored_profs.getint('version', default=0) if self.stored_profs else 0  # 1422
            self.print_msg('CK_BED_MESH', 'Now Version=%d, Cfg Version=%d' % (PR_VERSION, cfg_version))  # 1423
            err_cnt = err_cnt if cfg_version == PR_VERSION else 4             # 1424
            if err_cnt < 2:                                                   # 1425
                self.print_msg('DEBUG', 'check_bed_mesh: Pass!!')             # 1426
                return True                                                   # 1427
            self.print_msg('DEBUG', 'check_bed_mesh: Due to the great change of the hot bed or version, it needs to be re-leveled. '
                           + str([[errs[0], errs[1], errs[2], errs[3]], self.check_bed_mesh_max_err]))  # 1428
        if self.use_adc:                                                      # 1430
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)           # 1431
        self.gcode.run_script_from_command('BED_MESH_CALIBRATE')              # 1432
        configfile = self.printer.lookup_object('configfile')                 # 1434
        configfile.set('prtouch default', 'version', PR_VERSION)              # 1435
        self.gcode.run_script_from_command('CXSAVE_CONFIG')                   # 1437
        return True

    # ------------------------------------------------------------------
    # clear_nozzle  (def at 1442, body 1443..1511)
    # Validated against traces/trace_NCLR.log (full run, clr_noz_quick=False).
    # ------------------------------------------------------------------
    def clear_nozzle(self, hot_min_temp, hot_max_temp, bed_max_temp):
        self.print_msg('CLEAR_NOZZLE', 'Start clear_nozzle(), hot_min_temp=%.2f, hot_max_temp=%.2f, bed_max_temp=%.2f' % (hot_min_temp, hot_max_temp, bed_max_temp))  # 1443
        min_x, min_y = self.clr_noz_start_x, self.clr_noz_start_y                    # 1444
        self.set_bed_temps(temp=bed_max_temp, wait=False)                            # 1446
        self.set_hot_temps(temp=hot_min_temp, wait=False, err=10)                    # 1447
        self.ck_g28ed()                                                              # 1448
        mesh = self.bed_mesh.get_mesh()                                              # 1449
        self.bed_mesh.set_mesh(None)                                                 # 1450
        self.set_step_par(load_sys=False)                                            # 1451
        random.seed(time.time())                                                     # 1452
        cur_pos = self.toolhead.get_position()                                       # 1453
        src_pos = [min_x + random.uniform(0, self.clr_noz_len_x - self.pa_clr_dis_mm - 5),
                   min_y + random.uniform(0, self.clr_noz_len_y),
                   self.bed_max_err, cur_pos[3]]                                     # 1454-1455
        end_pos = [src_pos[0] + self.pa_clr_dis_mm, src_pos[1], src_pos[2], src_pos[3]]  # 1456
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=10)                     # 1457
        self.set_hot_temps(temp=hot_min_temp + 40, wait=False, err=10)               # 1458

        self.print_msg('CLEAR_NOZZLE', 'Start Src Pos Probe...')                     # 1460
        self.move([src_pos[0], src_pos[1], src_pos[2]], self.rdy_xy_spd)            # 1461
        self.z_set_status = True                                                     # 1462
        for i in range(3):                                                           # 1463
            src_pos[2], self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)  # 1464
            if not self.run_step_prtouch_flag:                                       # 1465
                self.print_msg('WHY ERROR', 'FUN:clear_nozzle run_step_prtouch() ERROR')  # 1466
                return False                                                         # 1467
            if src_pos[2] <= self.min_z:                                             # 1468
                self.nozzle_clear_z_out_of_range = True                              # 1469
                self.print_msg('WHY DEBUG', 'nozzle clear src_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!'.format(src_pos[2], self.min_z, self.nozzle_clear_z_out_of_range))  # 1470
                continue
            break
        if self.use_adc:                                                             # 1473
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)                  # 1474

        self.print_msg('CLEAR_NOZZLE', 'Start End Pos Probe...')                     # 1476
        self.move([end_pos[0], end_pos[1], end_pos[2]], self.rdy_xy_spd)            # 1477
        if self.use_adc:                                                             # 1478
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)                  # 1479
        self.set_fan_speed('fan', 0.0)                                               # 1480
        for i in range(3):                                                           # 1481
            end_pos[2], self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)  # 1482
            if not self.run_step_prtouch_flag:                                       # 1483
                self.print_msg('WHY ERROR', 'FUN:clear_nozzle run_step_prtouch() ERROR')  # 1484
                return False                                                         # 1485
            if end_pos[2] <= self.min_z:                                             # 1486
                self.nozzle_clear_z_out_of_range = True                              # 1487
                self.print_msg('WHY DEBUG', 'nozzle clear end_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!'.format(end_pos[2], self.min_z, self.nozzle_clear_z_out_of_range))  # 1488
                continue
            break
        self.z_set_status = False                                                    # 1491

        self.print_msg('CLEAR_NOZZLE', 'Down And Wait Temp For %.2f...' % hot_max_temp)  # 1493
        self.move([src_pos[0], src_pos[1], self.bed_max_err], self.rdy_xy_spd)      # 1494
        self.move([src_pos[0], src_pos[1], src_pos[2] - self.pa_clr_down_mm], self.rdy_z_spd)  # 1495
        self.set_hot_temps(temp=hot_max_temp, wait=True, err=10)                     # 1496

        self.print_msg('CLEAR_NOZZLE', 'Start Clear The Noz...')                     # 1498
        self.move(end_pos[:2] + [end_pos[2] + self.pa_clr_down_mm], self.clr_xy_spd)  # 1499
        self.set_fan_speed('fan', 1.0)                                               # 1500

        self.print_msg('CLEAR_NOZZLE', 'Start Cool Down The Noz...')                 # 1502
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)                      # 1503

        self.print_msg('CLEAR_NOZZLE', 'Start Finish Clear...')                      # 1505
        self.move([end_pos[0] + self.pa_clr_dis_mm, end_pos[1], end_pos[2] + self.bed_max_err], self.clr_xy_spd)  # 1506
        self.set_fan_speed('fan', 0.0)                                               # 1507
        self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)                      # 1508
        self.set_step_par(load_sys=True)                                             # 1509
        self.bed_mesh.set_mesh(mesh)                                                 # 1510
        return True

    # ------------------------------------------------------------------
    # clear_nozzle_quick  (def at 1515, body 1516..1665)
    # UNTESTED: clr_noz_quick is False on this printer, so this method never ran in
    # any trace. Reconstructed from the decompile (FUN_000b6dd4) alone.
    # ------------------------------------------------------------------
    def clear_nozzle_quick(self, hot_min_temp, hot_max_temp, bed_max_temp):
        self.print_msg('CLEAR_NOZZLE_QUICK', 'Start clear_nozzle_quick(), hot_min_temp=%.2f, hot_max_temp=%.2f, bed_max_temp=%.2f' % (hot_min_temp, hot_max_temp, bed_max_temp))  # 1516
        min_x, min_y = self.clr_noz_start_x, self.clr_noz_start_y                    # 1517
        max_x = self.clr_noz_start_x + self.clr_noz_len_x                            # 1518
        self.set_bed_temps(temp=bed_max_temp, wait=False)                            # 1519
        self.set_hot_temps(temp=hot_min_temp, wait=False, err=10)                    # 1520
        self.ck_g28ed()                                                              # 1521
        mesh = self.bed_mesh.get_mesh()                                              # 1522
        self.bed_mesh.set_mesh(None)                                                 # 1523
        self.set_step_par(load_sys=True)                                             # 1525 (True here, unlike clear_nozzle)
        max_accel = self.toolhead.max_accel                                          # 1526
        max_accel_to_decel = self.toolhead.max_accel_to_decel                        # 1527
        square_corner_velocity = self.toolhead.square_corner_velocity                # 1528
        self.toolhead.max_accel = 20000                                              # 1529
        self.toolhead.max_accel_to_decel = 10000                                     # 1530
        self.toolhead.square_corner_velocity = self.clr_xy_quick_spd                 # 1531

        random.seed(time.time())                                                     # 1536
        cur_pos = self.toolhead.get_position()                                       # 1537
        src_pos = [min_x, min_y, self.bed_max_err, cur_pos[3]]                       # 1538
        end_pos = [max_x, min_y, src_pos[2], src_pos[3]]                             # 1539
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=10)                     # 1540
        self.set_hot_temps(temp=hot_min_temp + 40, wait=False, err=10)               # 1541

        self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Src Pos Probe...')               # 1543
        self.move([src_pos[0], src_pos[1], src_pos[2] + 1], self.rdy_xy_spd)        # 1544
        self.z_set_status = True                                                     # 1545
        for i in range(3):                                                           # 1546
            src_pos[2], self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)  # 1547
            if not self.run_step_prtouch_flag:                                       # 1548
                self.print_msg('ERROR', 'FUN:clear_nozzle_quick run_step_prtouch() ERROR')  # 1549
                return False                                                         # 1550
            if src_pos[2] <= self.min_z:                                             # 1551
                self.nozzle_clear_z_out_of_range = True                              # 1552
                self.print_msg('WHY DEBUG', 'nozzle clear src_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!'.format(src_pos[2], self.min_z, self.nozzle_clear_z_out_of_range))  # 1553
                continue
            break
        if self.use_adc:                                                             # 1556
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)                  # 1557
        self.gcode.respond_info('[CLEAR_NOZZLE_QUICK] src_pos[2]:{}'.format(src_pos[2]))  # 1558

        self.print_msg('CLEAR_NOZZLE_QUICK', 'Start End Pos Probe...')               # 1560
        self.move([end_pos[0], end_pos[1], end_pos[2] + 1], self.rdy_xy_spd)        # 1561
        if self.use_adc:                                                             # 1562
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)                  # 1563
        self.set_fan_speed('fan', 0.0)                                               # 1564
        for i in range(3):                                                           # 1566
            end_pos[2], self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)  # 1567
            if not self.run_step_prtouch_flag:                                       # 1568
                self.print_msg('ERROR', 'FUN:clear_nozzle_quick run_step_prtouch() ERROR')  # 1569
                return False                                                         # 1570
            if end_pos[2] <= self.min_z:                                             # 1571
                self.nozzle_clear_z_out_of_range = True                              # 1572
                self.print_msg('WHY DEBUG', 'nozzle clear end_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!'.format(end_pos[2], self.min_z, self.nozzle_clear_z_out_of_range))  # 1573
                continue
            break
        self.z_set_status = False                                                    # 1576
        self.gcode.respond_info('[CLEAR_NOZZLE_QUICK] end_pos[2]:{}'.format(end_pos[2]))  # 1578

        self.print_msg('CLEAR_NOZZLE_QUICK', 'move to start position... ')           # 1580
        if end_pos[2] > 1.6 and src_pos[2] > 1.6:                                    # 1581
            # Quick wipe: both probe points read > 1.6 mm (raised wipe target).
            self.move([end_pos[0], end_pos[1], end_pos[2] - 0.2], self.rdy_xy_spd)  # 1582
            self.set_hot_temps(temp=hot_max_temp, wait=True, err=10)                 # 1583
            react_dis = self.clr_quick_react_dis                                     # 1585
            rand_y = random.uniform(-1, 0.5)                                         # 1586
            self.move([end_pos[0], end_pos[1], end_pos[2] + 1], self.rdy_xy_spd)    # 1587
            self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] + 1], self.clr_xy_quick_spd)  # 1588
            self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] - self.clr_quick_high], self.rdy_z_spd)  # 1589
            self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Clear The Noz... 1')         # 1591
            for i in range(self.clr_quick_times):                                    # 1593
                self.move([src_pos[0] + react_dis, src_pos[1] + rand_y, src_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1594
                self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1595
                if i % 2 == 1:                                                       # 1596
                    self.move([end_pos[0] - react_dis, end_pos[1] + rand_y - 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1597
                    self.gcode.run_script_from_command('G4 S2')                      # 1598
                    self.gcode.run_script_from_command('M400')                       # 1599
                else:
                    self.move([end_pos[0] - react_dis, end_pos[1] + rand_y + 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1601
                    self.gcode.run_script_from_command('G4 S2')                      # 1602
            self.move([end_pos[0], end_pos[1], end_pos[2]], self.rdy_xy_spd)        # 1604
            self.move([end_pos[0], end_pos[1], end_pos[2] - 0.2], self.rdy_xy_spd)  # 1605
            self.set_fan_speed('fan', 1.0)                                           # 1607
            self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)                  # 1608
            self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Cool Down The Noz...')       # 1609
            self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Clear The Noz... 2')         # 1612
            self.move([end_pos[0], end_pos[1], end_pos[2] + 1], self.rdy_xy_spd)    # 1613
            self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] + 1], self.clr_xy_quick_spd)  # 1614
            self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] - self.clr_quick_high], self.rdy_z_spd)  # 1615
            # The original loop variable is a C-level counter (name unknown); the
            # parity test below reads the stale `i` left by the loop at 1593.
            for _ in range(3):                                                       # 1616
                self.move([src_pos[0] + react_dis, src_pos[1] + rand_y, src_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1617
                self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1618
                if i % 2 == 1:                                                       # 1619
                    self.move([end_pos[0] - react_dis, end_pos[1] + rand_y - 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1620
                    self.gcode.run_script_from_command('G4 S2')                      # 1621
                    self.gcode.run_script_from_command('M400')                       # 1622
                else:
                    self.move([end_pos[0] - react_dis, end_pos[1] + rand_y + 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1624
                    self.gcode.run_script_from_command('G4 S2')                      # 1625
            self.move([end_pos[0] - react_dis, end_pos[1] + rand_y, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)  # 1627
            self.move([end_pos[0] - react_dis, end_pos[1] + rand_y - 4, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd)  # 1628
            self.move([src_pos[0] + react_dis, src_pos[1] + rand_y - 4, 5], 600)    # 1630
            self.set_fan_speed('fan', 0.0)                                           # 1631
            self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)                  # 1632
            self.toolhead.square_corner_velocity = square_corner_velocity            # 1634
            self.toolhead.max_accel = max_accel                                      # 1635
            self.toolhead.max_accel_to_decel = max_accel_to_decel                    # 1636
            self.bed_mesh.set_mesh(mesh)                                             # 1637
            return True

        # Fallback: same bed scrape as clear_nozzle, at a random XY, reusing the Z
        # values probed above. Toolhead accel/scv are NOT restored on this path.
        min_x, min_y = self.clr_noz_start_x, self.clr_noz_start_y                    # 1640
        self.set_step_par(load_sys=False)                                            # 1641
        random.seed(time.time())                                                     # 1642
        src_pos = [min_x + random.uniform(0, self.clr_noz_len_x - self.pa_clr_dis_mm - 5),
                   min_y + random.uniform(0, self.clr_noz_len_y),
                   src_pos[2], src_pos[3]]                                           # 1643-1644
        end_pos = [src_pos[0] + self.pa_clr_dis_mm, src_pos[1], end_pos[2], end_pos[3]]  # 1645
        self.print_msg('CLEAR_NOZZLE', 'Down And Wait Temp For %.2f...' % hot_max_temp)  # 1647
        self.move([src_pos[0], src_pos[1], self.bed_max_err], self.rdy_xy_spd)      # 1648
        self.move([src_pos[0], src_pos[1], src_pos[2] - self.pa_clr_down_mm], self.rdy_z_spd)  # 1649
        self.set_hot_temps(temp=hot_max_temp, wait=True, err=10)                     # 1650
        self.print_msg('CLEAR_NOZZLE', 'Start Clear The Noz...')                     # 1652
        self.move(end_pos[:2] + [end_pos[2] + self.pa_clr_down_mm], self.clr_xy_spd)  # 1653
        self.set_fan_speed('fan', 1.0)                                               # 1654
        self.print_msg('CLEAR_NOZZLE', 'Start Cool Down The Noz...')                 # 1656
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)                      # 1657
        self.print_msg('CLEAR_NOZZLE', 'Start Finish Clear...')                      # 1659
        self.move([end_pos[0] + self.pa_clr_dis_mm, end_pos[1], end_pos[2] + self.bed_max_err], self.clr_xy_spd)  # 1660
        self.set_fan_speed('fan', 0.0)                                               # 1661
        self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)                      # 1662
        self.set_step_par(load_sys=True)                                             # 1663
        self.bed_mesh.set_mesh(mesh)                                                 # 1664
        return True

    # ------------------------------------------------------------------
    # run_G28_Z  (py lines 1670-1848; body FUN_000d8704)
    # Called by homing.py PrinterHoming.cmd_G28 for axis Z as
    # mcu_probe.run_G28_Z(); a falsy return makes homing.py print key581.
    # cmd_ACCURATE_HOME_Z calls run_G28_Z(True).
    # Confidence: high for every statement checked against trace_G28.log and
    # trace_AHZ.log; retry/ramp/error branches (1741-1747, 1783-1804,
    # 1835-1838) are decompile-only; the step-loss branch (1751-1763) was
    # settled from the MIPS disassembly (see the comment there).
    # ------------------------------------------------------------------
    def run_G28_Z(self, accurate=False):
        self.print_msg('RUN_G28_Z', 'Start run_G28_Z({})...'.format(accurate))    # 1671
        self.enable_steps()                                                       # 1672
        self.get_mm_per_step()                                                    # 1673
        self.g29_cnt = 0                                                          # 1674
        self.jump_probe_ready = False                                             # 1675
        self.After_G28Z_skip_bed_tilt_flag = True                                 # 1676
        self.toolhead.wait_moves()                                                # 1677
        if not self.bed_mesh.get_mesh() and self.bed_mesh.pmgr.profiles.get('default', None):  # 1678
            self.gcode.run_script_from_command('BED_MESH_PROFILE LOAD="default"')  # 1679
        mesh = self.bed_mesh.get_mesh()                                           # 1680
        self.bed_mesh.set_mesh(None)                                              # 1681
        self.set_step_par(load_sys=False)                                         # 1682
        now_pos = self.toolhead.get_position()                                    # 1683
        self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])  # 1684
        min_x, min_y = self.bed_mesh.bmc.mesh_min                                 # 1685 (unused)
        max_x, max_y = self.bed_mesh.bmc.mesh_max                                 # 1686 (unused)
        home_pos = [self.home_x, self.home_y, now_pos[2], now_pos[3]]             # 1688
        target_temp = self.heater_hot.target_temp                                 # 1690
        if self.g28_wait_cool_down and self.heater_hot.smoothed_temp > (self.hot_min_temp + 5):  # 1691
            self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to cool down[%.2f -> %.2f]...'
                           % (target_temp, self.hot_min_temp))                    # 1692
            self.set_fan_speed('fan', 1.0)                                        # 1693
            self.set_hot_temps(temp=self.hot_min_temp, wait=True, err=5)          # 1694
            self.set_fan_speed('fan', 0.0)                                        # 1695
        random.seed(int(time.time()))                                             # 1697
        now_pos_sta0 = [home_pos[0] + (1 if int(time.time() * 1000) % 2 == 0 else -1) * random.uniform(4, 8),   # 1698
                        home_pos[1] + (1 if int(time.time() * 100) % 2 == 0 else -1) * random.uniform(4, 8),    # 1699
                        0, home_pos[3]]
        pos2 = [home_pos[0] + random.uniform(-1.0, 1.0),
                home_pos[1] + random.uniform(-1.0, 1.0),
                self.best_above_z * 2, home_pos[3]]                               # 1700
        if self.use_adc:                                                          # 1701
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)              # 1702
        self.set_fan_speed('fan', 0.0)                                            # 1703
        self.move(now_pos_sta0, self.rdy_xy_spd / 5)                              # 1704
        if not self.env_self_check():                                             # 1706
            self.print_msg('WHY ERROR', 'FUN:G28_Z env_self_check() ERROR')       # 1707
            return False
        abs_mms = []                                                              # 1710
        use_tri_times = 0                                                         # 1711
        z_down_spd = self.tri_z_down_spd[0]                                       # 1712
        hftr_cut = self.tri_hftr_cut[0]                                           # 1713
        lftr_k1 = self.tri_lftr_k1[0]                                             # 1714
        min_hold = self.tri_min_hold[0]                                           # 1715
        max_hold = self.tri_max_hold[0]                                           # 1716
        no_tri_cnt = 0                                                            # 1717
        all_mms = []                                                              # 1719
        ck_step_err_cnt = 0                                                       # 1720
        while True:                                                               # ~1722
            self.print_msg('RUN_G28_Z',
                           'use_tri_times=%d, use_tri_z_down_spd=%.2f, use_tri_hftr_cut=%.2f, '
                           'use_tri_lftr_k1=%.2f, use_tri_min_hold=%.0f, use_tri_max_hold=%.0f'
                           % (use_tri_times, z_down_spd, hftr_cut, lftr_k1, min_hold, max_hold))  # 1723-1724
            self.step_res, self.pres_res = [], []                                 # 1725
            self.enable_steps()                                                   # 1726
            self.print_msg('RUN_G28_Z', 'Start Coarse Probe, Test Index=%d...' % use_tri_times)  # 1727
            self.step_res, self.pres_res = [], []                                 # 1728
            res = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])             # 1729
            self.print_msg('AVGS_RESAULT', str(res))                              # 1730
            spd_k = 1.2 if self.use_adc else 2.0
            step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(self.max_z * 1.2, z_down_spd * spd_k)  # 1731
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms,
                                              self.tri_need_cnt, int(hftr_cut * 1000), int(lftr_k1 * 1000),
                                              int(min_hold), int(max_hold)])      # 1732-1733
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt, step_us,
                                              acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 1734
            t_start = time.time()                                                 # 1735
            while (time.time() - t_start) < (self.max_z * 2.0) / (z_down_spd * (1.2 if self.use_adc else 2.0)) \
                    and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):  # 1736
                self.delay_s(0.01)                                                # 1737
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0,
                                              self.low_spd_nul, self.send_step_duty, 0])  # 1738
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])  # 1739
            if len(self.pres_res) == 0 or len(self.step_res) == 0:                # 1741
                if no_tri_cnt < 2:                                                # 1742
                    no_tri_cnt += 1                                               # 1743
                    self.toolhead.set_position(now_pos_sta0[:2] + [0, now_pos_sta0[3]], homing_axes=[2])  # 1744
                    self.move(now_pos_sta0[:2] + [self.best_above_z * 2, now_pos_sta0[3]],
                              self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))  # 1745
                    continue
                self.delay_s(0.1)                                                 # 1747
            # A lost step stream does not abort: up to 2 losses fall through with
            # the out_mm = 999 sentinel (re-zero, lift, 999 enters the window and the
            # loop retries); the 3rd loss raises. ck_step_err_cnt is never reset.
            # Asm (file VA; Ghidra = +0x10000): int_999 loaded at 0xd0400
            # (s2); `count > 2` false branches at 0xd0798 to 0xd1aa4, which stores
            # s2 into the out_mm slot and jumps to 0xd10cc (L1765 set_position);
            # Py_False is returned only inside the `> 2` block (0xd0c70).
            out_mm = 999                                                          # ~1751
            if not self.ck_and_manual_get_step():                                 # 1752
                ck_step_err_cnt += 1                                              # 1753
                # third positional arg is the counter, not a .format() argument (as in the binary)
                self.print_msg('WHY ERROR', 'FUN:G28_Z ck_and_manual_get_step() ERROR Times:{}',
                               ck_step_err_cnt)                                   # 1754
                if ck_step_err_cnt > 2:                                           # 1756
                    self.ck_and_raise_error(True, PR_ERR_CODE_STEP_LOST_RUN_DATA,
                                            ['step_res len:{}'.format(len(self.step_res))])  # 1757
                    return False
            else:
                out_mm = 0                                                        # ~1760
                if len(self.pres_res) != MAX_BUF_LEN:                             # 1761
                    out_mm = self.toolhead.get_position()[2] - \
                        (step_cnt - self.step_res[-1]['step']) * self.mm_per_step  # 1762
                else:
                    out_mm = self.cal_tri_data(step_cnt, self.toolhead.get_position()[2],
                                               self.step_res, self.pres_res)      # 1763
            self.toolhead.set_position(now_pos_sta0[:2] + [0, now_pos_sta0[3]], homing_axes=[2])  # 1765
            self.move(now_pos_sta0[:2] + [self.best_above_z * 2, now_pos_sta0[3]],
                      self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))         # 1771
            all_mms.append(out_mm)                                                # 1773
            abs_mms.append(math.fabs(out_mm))                                     # 1774
            if len(abs_mms) > 3:                                                  # 1775
                del abs_mms[0]                                                    # 1776
            sort_mms = [m for m in abs_mms]                                       # 1777
            sort_mms.sort()                                                       # 1778
            self.print_ary('G28_FIRST_MMS', sort_mms, len(sort_mms), 3)           # 1779
            if len(sort_mms) == 3 and (sort_mms[0] + sort_mms[1]) < 2.0:          # 1780
                break
            use_tri_times += 1                                                    # 1783
            if self.tri_try_max_times != 0:                                       # 1784
                if use_tri_times == self.tri_try_max_times:                       # 1785
                    self.print_msg('WHY ERROR',
                                   'FUN:G28_Z Coarse Probe Out of max {} times:{}. all data:{}'.format(
                                       self.tri_try_max_times, use_tri_times, all_mms))  # 1786
                    self.ck_and_raise_error(True, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)  # 1788
                    return False
            if use_tri_times <= 20 and use_tri_times >= 5:                        # 1790
                ratio = (use_tri_times - 5) / 15.0                                # 1791
                z_down_spd = (self.tri_z_down_spd[1] - self.tri_z_down_spd[0]) * ratio + self.tri_z_down_spd[0]  # 1792
                hftr_cut = (self.tri_hftr_cut[1] - self.tri_hftr_cut[0]) * ratio + self.tri_hftr_cut[0]  # 1793
                lftr_k1 = (self.tri_lftr_k1[1] - self.tri_lftr_k1[0]) * ratio + self.tri_lftr_k1[0]  # 1794
                min_hold = (self.tri_min_hold[1] - self.tri_min_hold[0]) * ratio + self.tri_min_hold[0]  # 1795
                max_hold = (self.tri_max_hold[1] - self.tri_max_hold[0]) * ratio + self.tri_max_hold[0]  # 1796
            if use_tri_times % 5 == 0:                                            # 1797
                self.toolhead.set_position(now_pos_sta0[:2] + [0, now_pos_sta0[3]], homing_axes=[2])  # 1798
                self.move(now_pos_sta0[:2] + [self.best_above_z * 2, now_pos_sta0[3]],
                          self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))     # 1799
                self.shake_motor(self.shake_cnt)                                  # 1800
            if use_tri_times == 19:                                               # 1801
                self.print_msg('WHY ERROR',
                               'FUN:G28_Z Coarse Probe Out of max 19 times:{}. all data:{}'.format(
                                   use_tri_times, all_mms))                       # 1802
                self.ck_and_raise_error(True, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)  # 1804
                return False
        if not accurate:                                                          # 1806
            if self.g28_wait_cool_down:                                           # 1807
                self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to recovery[%.2f -> %.2f]...'
                               % (self.hot_min_temp, target_temp))                # 1808
                self.set_hot_temps(temp=target_temp, wait=False, err=5)           # 1809
            return True
        index = 0                                                                 # ~1812
        while True:
            self.print_msg('RUN_G28_Z', 'Start Precision Probe, index=%d...' % index)  # 1814
            self.move(self.toolhead.get_position()[:2] + [pos2[2]], self.rdy_z_spd)  # 1815
            self.move(pos2, self.rdy_xy_spd / 5)                                  # 1816
            self.shake_motor(self.shake_cnt)                                      # 1817
            self.z_set_status = True                                              # 1818
            res_z, self.run_step_prtouch_flag = self.run_step_prtouch(
                self.max_z * 1.2, self.probe_min_3err, False, 5, 3, True)         # 1819
            self.z_set_status = False                                             # 1820
            if not self.run_step_prtouch_flag:                                    # 1821
                self.print_msg('WHY ERROR', 'FUN:run_G28_Z run_step_prtouch() ERROR')  # 1822
                return False
            self.toolhead.set_position(pos2[:2] + [self.toolhead.get_position()[2] - res_z, home_pos[3]],
                                       homing_axes=[2])                           # 1824
            self.print_msg('RUN_G28_Z', 'Start Result Check Probe...')            # 1825
            self.move(pos2[:2] + [self.bed_max_err, home_pos[3]], self.rdy_z_spd)  # 1826
            ck_z, self.run_step_prtouch_flag = self.run_step_prtouch(
                self.max_z * 1.2, self.probe_min_3err, False, 3, 3, True)         # 1827
            if not self.run_step_prtouch_flag:                                    # 1828
                self.print_msg('WHY ERROR', 'FUN:run_G28_Z run_step_prtouch() ERROR')  # 1829
                return False
            index += 1                                                            # 1831
            if math.fabs(ck_z) < 1.0:                                             # 1832
                break
            if index == 2:                                                        # 1835
                self.print_msg('WHY ERROR',
                               'FUN:G28_Z Precision Probe Out of max 2 times:{}. ck_z data:{}'.format(
                                   index, ck_z))                                  # 1836
                self.ck_and_raise_error(True, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)  # 1837
                return False  # value after the (raising) call not recovered; see notes
        if self.g28_wait_cool_down:                                               # 1841
            self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to recovery[%.2f -> %.2f]...'
                           % (self.hot_min_temp, target_temp))                    # 1842
            self.set_hot_temps(temp=target_temp, wait=target_temp > self.hot_min_temp, err=5)  # 1843
        if self.use_adc:                                                          # 1844
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)              # 1845
        self.set_step_par(load_sys=True)                                          # 1846
        self.bed_mesh.set_mesh(mesh)                                              # 1847
        return True                                                               # 1848

    # ------------------------------------------------------------------
    # run_re_g29s  (py lines 1850-1927; wrapper FUN_0004ab08, body FUN_00042da0)
    # Re-probes every point already in bed_mesh.bmc.probe_helper.results plus
    # last_point (appended temporarily), after a fresh accurate Z home and
    # probe_ready. If a home step-loss check then fails, it re-homes and
    # re-probes every point again, re-zeroing Z on a home probe before each.
    # Returns the re-probed Z of last_point (results[-1][2], popped again), or
    # False when a probe fails.
    # No caller: nothing in the binary or in the stock klippy tree calls it
    # (n_run_re_g29s only appears in the module name table). NOT traced; this is
    # decompile-only. Confidence: medium-high on statement order (Cython line
    # numbers), the nesting (only the shake is under each `if i % x_cnt == 0`),
    # and the call arguments. Lines 1860-1869 and 1914-1924 emit no code
    # (comments or blank lines in the original).
    # ------------------------------------------------------------------
    def run_re_g29s(self, last_point):
        self.print_msg('RUN_RE_G29S', 'Start run_re_g29s()...')                   # 1851
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']                          # 1853
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']                          # 1854 (unused)
        results = self.bed_mesh.bmc.probe_helper.results                          # 1856
        results.append(last_point)                                                # 1857
        now_pos = self.toolhead.get_position()                                    # 1859
        self.move(now_pos[:2] + [5, now_pos[3]], self.rdy_z_spd)                  # 1870
        self.run_G28_Z(True)                                                      # 1872
        self.probe_ready()                                                        # 1873
        for i in range(len(results)):                                             # 1874
            self.print_msg('RUN_RE_G29S_ONE', 'Re Probe = ' + str(results[i]))    # 1875
            if i % x_cnt == 0:                                                    # 1876
                self.shake_motor(self.shake_cnt / 4)                              # 1877
            self.run_to_next(results[i], True)                                    # 1878
            results[i][2], self.run_step_prtouch_flag = self.run_step_prtouch(    # 1879
                self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            if not self.run_step_prtouch_flag:                                    # 1880
                self.print_msg('WHY ERROR', 'FUN:run_re_g29s run_step_prtouch() ERROR')  # 1881
                return False                                                      # 1882
        self.print_msg('RUN_RE_G29S', 'Start Step Lost Check...')                 # 1884
        self.move([self.home_x, self.home_y,                                      # 1885
                   self.get_best_rdy_z(self.home_x, self.home_y, self.rdy_pos) + self.best_above_z],
                  self.rdy_xy_spd)
        home_z, self.run_step_prtouch_flag = self.run_step_prtouch(               # 1886
            self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
        if not self.run_step_prtouch_flag:                                        # 1887
            self.print_msg('WHY ERROR', 'FUN:run_re_g29s run_step_prtouch() ERROR')  # 1888
            return False                                                          # 1889
        self.print_msg('RUN_RE_G29S', 'Step Lost Check, need=0, tri=%.2f' % home_z)  # 1890
        if math.fabs(home_z) > self.lost_step_dis and self.lost_step_dis > 0:     # 1892
            self.run_G28_Z(True)                                                  # 1893
            self.probe_ready()                                                    # 1894
            for i in range(len(results)):                                         # 1895
                self.print_msg('RUN_RE_G29S_TWO', 'Re Probe = ' + str(results[i]))  # 1896
                if i % x_cnt == 0:                                                # 1897
                    self.shake_motor(self.shake_cnt / 4)                          # 1898
                now_pos = self.toolhead.get_position()                            # 1899
                self.move(now_pos[:2] + [5], self.rdy_z_spd)                      # 1900
                self.move([self.home_x, self.home_y, 5], self.rdy_xy_spd)         # 1901
                tri_z, self.run_step_prtouch_flag = self.run_step_prtouch(        # 1902
                    self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
                if not self.run_step_prtouch_flag:                                # 1903
                    self.print_msg('WHY ERROR', 'FUN:run_re_g29s run_step_prtouch() ERROR')  # 1904
                    return False                                                  # 1905
                now_pos = self.toolhead.get_position()                            # 1906
                self.toolhead.set_position(now_pos[:2] + [now_pos[2] - tri_z, now_pos[3]],  # 1907
                                           homing_axes=[2])
                self.run_to_next(results[i], True)                                # 1908
                results[i][2], self.run_step_prtouch_flag = self.run_step_prtouch(  # 1909
                    self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
                if not self.run_step_prtouch_flag:                                # 1910
                    self.print_msg('WHY ERROR', 'FUN:run_re_g29s run_step_prtouch() ERROR')  # 1911
                    return False                                                  # 1912
            self.run_G28_Z(True)                                                  # 1913
        res_z = results[-1][2]                                                    # 1925
        del results[-1]                                                           # 1926
        return res_z                                                              # 1927

    # ------------------------------------------------------------------
    # run_G29_Z  (py lines 1930-1980; M_run_G29_Z, METH_O, body inline)
    # One bed-mesh point. Returns now_pos with now_pos[2] = probed Z (homing.py
    # uses it as epos), or False on a probe failure.
    # g29_cnt == 0: fan off, probe_ready (4 corners), move to the point at the
    # tilt-corrected height, shake. Last point (g29_cnt == x_count*y_count):
    # step-loss check probe at home_x/home_y, bed_mesh_post_proc(now_pos)
    # (which may rewrite now_pos[2] in place), lift to Z=10, move to home.
    # Confidence: high; all 25 traced calls replay (recon/check_g29flow.py).
    # Not traced: use_adc branches (1941-1942, 1977-1978), the False returns,
    # the lost-step raise (1968-1970), the bed_mesh_post_proc failure (1973).
    # ------------------------------------------------------------------
    def run_G29_Z(self):
        self.print_msg('RUN_G29_Z', 'Start run_G29_Z()...')                       # 1931
        self.set_step_par(load_sys=False)                                         # 1932
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']                          # 1933
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']                          # 1934
        self.toolhead.wait_moves()                                                # 1935
        now_pos = self.toolhead.get_position()                                    # 1936
        if int(self.g29_cnt) % int(x_cnt) == 0:                                   # 1937
            self.shake_motor(self.shake_cnt / 4)                                  # 1938
        self.print_msg('RUN_G29_Z', 'g29_cnt=%d, x=%.2f, y=%.2f, z=%.2f' % (      # 1939
            self.g29_cnt, now_pos[0], now_pos[1], now_pos[2]))
        if self.g29_cnt == 0:                                                     # 1940
            if self.use_adc:                                                      # 1941
                self.set_fan_speed('heater_fan', self.fan_heat_min_spd)           # 1942
            else:
                self.set_fan_speed('fan', 0.0)                                    # 1943
            if self.probe_ready():                                                # 1944
                self.run_to_next(now_pos, True)                                   # 1945
                self.shake_motor(self.shake_cnt / 4)                              # 1946
            elif not self.run_step_prtouch_flag:                                  # 1947
                self.print_msg('WHY ERROR', 'FUN:run_G29_Z probe_ready() ERROR')  # 1948
                return False                                                      # 1949
        self.g29_cnt += 1                                                         # 1950
        now_pos[2], self.run_step_prtouch_flag = self.run_step_prtouch(           # 1952
            self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True, re_g28=True)
        if not self.run_step_prtouch_flag:                                        # 1953
            self.print_msg('WHY ERROR', 'FUN:run_G29_Z run_step_prtouch() ERROR')  # 1954
            return False                                                          # 1955
        if self.g29_cnt == x_cnt * y_cnt:                                         # 1957
            self.g29_cnt = 0                                                      # 1958
            self.print_msg('RUN_G29_Z', 'Start Step Lost Check...')               # 1959
            self.move([self.home_x, self.home_y,                                  # 1960
                       self.get_best_rdy_z(self.home_x, self.home_y, self.rdy_pos) + self.best_above_z],
                      self.rdy_xy_spd)
            home_z, self.run_step_prtouch_flag = self.run_step_prtouch(           # 1961
                self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            self.print_msg('RUN_G29_Z', 'Step Lost Check, need=0, tri=%.2f' % home_z)  # 1962
            if not self.run_step_prtouch_flag:                                    # 1963
                self.print_msg('WHY ERROR', 'FUN:run_G29_Z run_step_prtouch() ERROR')  # 1964
                return False                                                      # 1965
            if math.fabs(home_z) > self.lost_step_dis and self.lost_step_dis > 0:  # 1968
                self.print_msg('WHY ERROR',                                       # 1969
                               'FUN:run_G29_Z check last point res_z:({}) out of lost_step_dis:{}'.format(
                                   math.fabs(home_z), self.lost_step_dis))
                self.ck_and_raise_error(True, PR_ERR_CODE_HAVE_LOST_STEP, [       # 1970
                    'RUN_G29_Z check last point res_z:({}) out of lost_step_dis:{}'.format(
                        math.fabs(home_z), self.lost_step_dis)])
            if not self.bed_mesh_post_proc(now_pos):                              # 1972
                self.print_msg('WHY ERROR', 'FUN:run_G29_Z bed_mesh_post_proc ERROR')  # 1973
                return False                                                      # 1974
            self.move(self.toolhead.get_position()[:2] + [10], self.rdy_z_spd)    # 1975
            self.move([self.home_x, self.home_y, 10], self.rdy_xy_spd)            # 1976
            if self.use_adc:                                                      # 1977
                self.set_fan_speed('heater_fan', self.fan_heat_max_spd)           # 1978
        self.set_step_par(load_sys=True)                                          # 1979
        return now_pos                                                            # 1980

    cmd_READ_PRES_help = 'Read The Press Vals.'

    # L1984-2006. UNTESTED (no trace calls cmd_READ_PRES).
    # Parameters: C (int, default 1) = sample count, F (int, default 0) = run
    # deal_avgs (8 samples) first. No min/max bounds in the binary.
    def cmd_READ_PRES(self, gcmd):
        self.print_msg('VERSION', 'ARM(PRTH)=%s, MCU(PRES)=%s, MCU(STEP)=%s'
                       % (self.ver_prth, self.ver_pres, self.ver_step))         # 1985
        self.pres_res = []                                                       # 1986
        cnt = gcmd.get_int('C', 1)                                               # 1987
        deal_avgs = gcmd.get_int('F', 0)                                         # 1988
        if deal_avgs:                                                            # 1989
            res = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])            # 1990
            self.print_msg('AVGS_RESAULT', str(res))                             # 1991
        self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, cnt])  # 1992
        t_start = time.time()                                                    # 1993
        while (time.time() - t_start) < 1.5 * (self.tri_acq_ms / 1000.0) * cnt \
                and len(self.pres_res) < cnt:                                    # 1994
            self.delay_s(0.01)                                                   # 1995
        pres_vals = [[], [], [], []]                                             # 1996
        ticks = []                                                               # 1997
        for i in range(len(self.pres_res)):                                      # 1998
            ticks.append(self.pres_res[i]['tick'] / 10000.0)                     # 1999
            for j in range(self.pres_cnt):                                       # 2000
                pres_vals[j].append(self.pres_res[i]['ch%d' % j])                # 2001
        self.print_ary('READ_PRES_TICKS', ticks, len(ticks), 3, True)            # 2003
        for i in range(self.pres_cnt):                                           # 2004
            self.print_ary('READ_PRES_CH%d' % i, pres_vals[i], len(pres_vals[i]), 0, True)  # 2005
            self.send_wave_tri(i, pres_vals[i])                                  # 2006

    cmd_TEST_SWAP_help = 'Test The Swap Pin.'

    # L2011-2020. UNTESTED. No G-code parameters.
    # NOTE (faithful to the binary): read_swap_prtouch is sent with
    # self.pres_oid, not self.step_oid. read_swap_prtouch_cmd lives on the step
    # MCU (looked up with oid=step_oid); env_self_check sends step_oid there
    # (trace G28:139 sends [5]). Here the binary only references n_pres_oid
    # (oid 0 on the K1). On Klipper firmware an oid of the wrong type makes the
    # MCU shut down, so this command may be unsafe to run as-is.
    def cmd_TEST_SWAP(self, gcmd):
        params1 = self.write_swap_prtouch_cmd.send([self.pres_oid, 1])          # 2012
        params1 = self.read_swap_prtouch_cmd.send([self.pres_oid])              # 2013
        self.write_swap_prtouch_cmd.send([self.pres_oid, 0])                    # 2014
        params0 = self.read_swap_prtouch_cmd.send([self.pres_oid])              # 2015
        self.print_msg('SWAP_TEST', 'params0:{} params1:{}'.format(params0, params1))  # 2016
        if not params0 or not params1 or params0['sta'] != 0 or params1['sta'] != 1:  # 2017
            self.print_msg('SWAP_TEST', '!!!Swap Test ERROR!!!', True)          # 2018
        else:
            self.print_msg('SWAP_TEST', '---Swap Test Success---', True)        # 2020

    cmd_DEAL_AVGS_help = 'Read And Cal The Avgs.'

    # L2024-2027. UNTESTED. Parameter: C (int, default 8), no bounds.
    def cmd_DEAL_AVGS(self, gcmd):
        cnt = gcmd.get_int('C', 8)                                               # 2025
        res = self.deal_avgs_prtouch_cmd.send([self.pres_oid, cnt])             # 2026
        self.print_msg('AVGS_RESAULT', str(res), True)                          # 2027

    cmd_NOZZLE_CLEAR_help = 'Clear the nozzle on bed.'

    def cmd_NOZZLE_CLEAR(self, gcmd):
        hot_min_temp = gcmd.get_float('HOT_MIN_TEMP', self.hot_min_temp)            # 2032
        hot_max_temp = gcmd.get_float('HOT_MAX_TEMP', self.hot_max_temp)            # 2033
        bed_max_temp = gcmd.get_float('BED_MAX_TEMP', self.bed_max_temp)            # 2034
        self.gcode.run_script_from_command('RESTORE_E_CURRENT')                      # 2035
        if self.clr_noz_quick:                                                       # 2036
            self.clear_nozzle_quick(hot_min_temp, hot_max_temp, bed_max_temp)        # 2037
        else:
            self.clear_nozzle(hot_min_temp, hot_max_temp, bed_max_temp)              # 2039

    cmd_CHECK_BED_MESH_help = 'Check the bed mesh.'

    def cmd_CHECK_BED_MESH(self, gcmd):                      # 2042-2043
        self.check_bed_mesh(gcmd.get_int('AUTO_G29', 0) > 0)

    cmd_START_STEP_PRTOUCH_help = 'Start the step prtouch.'

    # L2047-2059. UNTESTED. Raw step-MCU move with the probe armed.
    # Parameters: DIR (int, default 0), SPD (float, default 10),
    # DIS (float, default 10). No bounds in the binary.
    def cmd_START_STEP_PRTOUCH(self, gcmd):
        self.enable_steps()                                                      # 2048
        self.get_mm_per_step()                                                   # 2049
        dir = gcmd.get_int('DIR', 0)                                             # 2050
        spd = gcmd.get_float('SPD', 10)                                          # 2051
        dis = gcmd.get_float('DIS', 10)                                          # 2052
        self.step_res = []                                                       # 2053
        step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(dis, spd)            # 2054
        self.start_step_prtouch_cmd.send([self.step_oid, dir, self.tri_send_ms, step_cnt, step_us,
                                          acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 2055
        t_start = time.time()                                                    # 2056
        while (time.time() - t_start) < dis / spd + 5 \
                and len(self.step_res) != MAX_BUF_LEN:                           # 2057
            self.delay_s(0.01)                                                   # 2058
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0,
                                          self.low_spd_nul, self.send_step_duty, 0])  # 2059

    cmd_PRTOUCH_READY_help = 'Test the ready point.'

    def cmd_PRTOUCH_READY(self, gcmd):                       # 2063-2064
        self.probe_ready()

    cmd_SAFE_MOVE_Z_help = 'Safe move z'

    def cmd_SAFE_MOVE_Z(self, gcmd):                         # 2068-2076
        self.v_sd.run_dis = 0.0                              # 2069
        sta = gcmd.get_int('STA', 0)                         # 2070
        dis = gcmd.get_float('DIS', 10)                      # 2071
        spd = gcmd.get_float('SPD', 5.0)                     # 2072
        rdo = gcmd.get_float('RDO', 1.0)                     # 2073
        if sta == 0:                                         # 2074
            self.run_sta_status = 0                          # 2075
        self.safe_move_z(sta, dis, spd, rdo)                 # 2076

    cmd_ACCURATE_HOME_Z_help = 'Coarse home z'

    def cmd_ACCURATE_HOME_Z(self, gcmd):                     # 2080-2082
        self.ck_g28ed()                                      # 2081
        self.run_G28_Z(True)                                 # 2082

    cmd_SAFE_DOWN_Z_help = 'Safe down z before G28'

    # ------------------------------------------------------------------
    # cmd_SAFE_DOWN_Z  (py lines 2085-2143; wrapper M_cmd_SAFE_DOWN_Z,
    # body FUN_000cb958)
    # SAFE_DOWN_Z [DOWN_DIS=5] [UP_DIS=0]. Raw MCU stepping of Z without the
    # toolhead: DOWN_DIS moves with direction 1 (bed away from nozzle), then
    # UP_DIS moves with direction 0 under pressure triggering. Each leg first
    # covers safe_z_dead_dis without the pressure MCU when the distance is
    # larger than safe_z_dead_dis + 1. The two legs are independent ifs.
    # UNTESTED: no trace covers this command.
    # NB: get_step_cnts is passed self.tri_z_down_spd and the start_pres list
    # uses self.tri_hftr_cut / tri_lftr_k1 / tri_min_hold / tri_max_hold
    # without indexing. With the stock config these are 2-tuples, so every
    # leg raises TypeError inside get_step_cnts (run_dis / run_spd) after
    # deal_avgs (triggered legs) and before any step command is sent. This is
    # faithful to the binary (no GetItem in FUN_000cb958); see notes.
    # The dead-zone start_step lists differ: the DOWN leg sends acc_ctl_cnt
    # in slot 5, the UP leg sends 0 (verified in the disassembly).
    # ------------------------------------------------------------------
    def cmd_SAFE_DOWN_Z(self, gcmd):
        self.get_mm_per_step()                                                # 2087
        self.enable_steps()                                                   # 2088
        down_dis = gcmd.get_float('DOWN_DIS', 5)                              # 2089
        up_dis = gcmd.get_float('UP_DIS', 0)                                  # 2090
        if down_dis != 0:                                                     # 2092
            if self.safe_z_dead_dis != 0 and down_dis > self.safe_z_dead_dis + 1:  # 2093
                self.step_res, self.pres_res = [], []                         # 2094
                step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(self.safe_z_dead_dis, self.tri_z_down_spd)  # 2095
                self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt, step_us,
                                                  acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 2096
                t_start = time.time()                                         # 2097
                while time.time() - t_start < self.safe_z_dead_dis / 1.0 + 10 and len(self.step_res) != MAX_BUF_LEN:  # 2098
                    self.delay_s(0.01)                                        # 2099
                self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0,
                                                  self.low_spd_nul, self.send_step_duty, 0])  # 2100
                self.ck_and_manual_get_step()                                 # 2101
                down_dis -= self.safe_z_dead_dis                              # 2102
            self.step_res, self.pres_res = [], []                             # 2103
            res = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])         # 2104
            self.print_msg('AVGS_RESAULT', str(res))                          # 2105
            step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(down_dis, self.tri_z_down_spd)  # 2106
            self.start_pres_prtouch_cmd.send([self.pres_oid, 1, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt,
                                              int(self.tri_hftr_cut * 1000), int(self.tri_lftr_k1 * 1000),   # 2108
                                              self.tri_min_hold if self.use_adc else self.tri_min_hold * 2,  # 2109
                                              self.tri_max_hold if self.use_adc else self.tri_max_hold * 2])  # 2107
            self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt, step_us,
                                              acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 2110
            t_start = time.time()                                             # 2111
            while time.time() - t_start < down_dis / self.tri_z_down_spd + 10 and len(self.step_res) != MAX_BUF_LEN:  # 2112
                self.delay_s(0.01)                                            # 2113
            self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0,
                                              self.low_spd_nul, self.send_step_duty, 0])  # 2114
            self.start_pres_prtouch_cmd.send([self.pres_oid, 1, 0, 0, 0, 0, 0, 0, 0])  # 2115
            self.print_res('cmd_SAFE_DOWN_Z')                                 # 2116
            self.ck_and_manual_get_step()                                     # 2117
        if up_dis != 0:                                                       # 2119
            if self.safe_z_dead_dis != 0 and up_dis > self.safe_z_dead_dis + 1:  # 2120
                self.step_res, self.pres_res = [], []                         # 2121
                step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(self.safe_z_dead_dis, self.tri_z_down_spd)  # 2122
                self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt, step_us,
                                                  0, self.low_spd_nul, self.send_step_duty, 0])  # 2123
                t_start = time.time()                                         # 2124
                while time.time() - t_start < self.safe_z_dead_dis / 1.0 + 10 and len(self.step_res) != MAX_BUF_LEN:  # 2125
                    self.delay_s(0.01)                                        # 2126
                self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0,
                                                  self.low_spd_nul, self.send_step_duty, 0])  # 2127
                self.ck_and_manual_get_step()                                 # 2128
                up_dis -= self.safe_z_dead_dis                                # 2129
            self.step_res, self.pres_res = [], []                             # 2130
            res = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])         # 2131
            self.print_msg('AVGS_RESAULT', str(res))                          # 2132
            step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(up_dis, self.tri_z_down_spd)  # 2133
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt,
                                              int(self.tri_hftr_cut * 1000), int(self.tri_lftr_k1 * 1000),   # 2135
                                              self.tri_min_hold if self.use_adc else self.tri_min_hold * 2,  # 2136
                                              self.tri_max_hold if self.use_adc else self.tri_max_hold * 2])  # 2134
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt, step_us,
                                              acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])  # 2137
            t_start = time.time()                                             # 2138
            while time.time() - t_start < up_dis / self.tri_z_down_spd + 10 and len(self.step_res) != MAX_BUF_LEN:  # 2139
                self.delay_s(0.01)                                            # 2140
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0,
                                              self.low_spd_nul, self.send_step_duty, 0])  # 2141
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])  # 2142
            self.ck_and_manual_get_step()                                     # 2143

    cmd_TRIG_TEST_help = 'Test The Tri is Normal'

    # L2147-2151. UNTESTED. Parameter: C (int, default 1), no bounds.
    def cmd_TRIG_TEST(self, gcmd):
        self.enable_steps()                                                      # 2148
        self.get_mm_per_step()                                                   # 2149
        cnt = gcmd.get_int('C', 1)                                               # 2150
        self.run_step_prtouch(20, 0, False, cnt, cnt, True)                      # 2151

    cmd_TRIG_BED_TEST_help = 'Test The Tri in bed mesh'

    # L2154-2238. UNTESTED. Probes every point of a bed_mesh-style grid and
    # logs the trigger heights.
    # Parameters: F (int, default 1; 1 = log to CSV, 0 = close any open log),
    # C (int, default 1; probe count per point), S (float, default 1.0; down
    # speed). No bounds in the binary.
    # NOTES (faithful to the binary):
    # - Line 2230 writes to self.save_pres_file unconditionally, so with F=0
    #   (save_pres_file None) the first point raises AttributeError. There is
    #   no try/finally: tri_z_down_spd/tri_z_up_spd and the cleared bed mesh are
    #   then not restored.
    # - The grid generation (2182-2208) is bed_mesh._generate_points without
    #   the round-bed branch; each point gets z = 2.
    def cmd_TRIG_BED_TEST(self, gcmd):
        if not self.bed_mesh.get_mesh() and self.bed_mesh.pmgr.profiles.get('default', None):  # 2156
            self.gcode.run_script_from_command('BED_MESH_PROFILE LOAD="default"')  # 2157
        mesh = self.bed_mesh.get_mesh()                                          # 2158
        self.bed_mesh.set_mesh(None)                                             # 2159
        now_time = datetime.datetime.now().strftime('[%Y-%m-%d_%H_%M_%S]')       # 2160
        mode = gcmd.get_int('F', 1)                                              # 2161
        cnt = gcmd.get_int('C', 1)                                               # 2162
        spd = gcmd.get_float('S', 1.0)                                           # 2163
        if mode == 1:                                                            # 2164
            path = '/usr/share/klipper/pres_data/'                               # 2165
            file_name = path + now_time + '_Tri_bedmesh_data_{}_{}.csv'.format(cnt, spd)  # 2166
            if not os.path.exists(path):                                         # 2168
                os.makedirs(path)                                                # 2169
            self.save_pres_file = open(file_name, mode='w')                      # 2170
            title = ''                                                           # 2172 (init)
            for i in range(cnt):                                                 # 2172
                title += ',Tri_Z{}'.format(i)                                    # 2173
            title += ',MAX,MIN,(MAX-MIN)'                                        # 2174
            self.save_pres_file.writelines('Time,Speed,Pos_X,Pos_Y,Pos_Z' + title + '\n')  # 2175
        elif mode == 0:                                                          # 2176
            if self.save_pres_file is not None:                                  # 2177
                self.save_pres_file.close()                                      # 2178
                self.save_pres_file = None                                       # 2179
        min_x, min_y = self.bed_mesh.bmc.mesh_min                                # 2182
        max_x, max_y = self.bed_mesh.bmc.mesh_max                                # 2183
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']                         # 2184
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']                         # 2185
        x_dist = (max_x - min_x) / (x_cnt - 1)                                   # 2187
        y_dist = (max_y - min_y) / (y_cnt - 1)                                   # 2188
        x_dist = math.floor(x_dist * 100) / 100                                  # 2190
        y_dist = math.floor(y_dist * 100) / 100                                  # 2191
        if x_dist < 1.0 or y_dist < 1.0:                                         # 2192
            raise self.printer.command_error(
                '{"code":"key555", "msg":"cmd_TRIG_BED_TEST bed_mesh: min/max points too close together", "values": []}')  # 2193
        max_x = min_x + x_dist * (x_cnt - 1)                                     # 2195
        pos_y = min_y                                                            # 2196
        points = []                                                              # 2197
        for i in range(y_cnt):                                                   # 2198
            for j in range(x_cnt):                                               # 2199
                if not i % 2:                                                    # 2200
                    pos_x = min_x + j * x_dist                                   # 2202
                else:
                    pos_x = max_x - j * x_dist                                   # 2205
                points.append([pos_x, pos_y, 2])                                 # 2207
            pos_y += y_dist                                                      # 2208
        self.enable_steps()                                                      # 2210
        self.get_mm_per_step()                                                   # 2211
        tri_z_down_spd_bak = self.tri_z_down_spd                                 # 2212
        self.tri_z_down_spd = (spd, spd)                                         # 2214
        tri_z_up_spd_bak = self.tri_z_up_spd                                     # 2215
        self.tri_z_up_spd = 5                                                    # 2216
        for i in range(len(points)):                                             # 2217
            self.run_to_next(points[i], True)                                    # 2218
            now_pos = self.toolhead.get_position()                               # 2219
            self.print_msg('WHY DEBUG', 'Start Tri Point:{} Pos:{}'.format(i, now_pos[:-1]))  # 2220
            self.run_step_prtouch(20, 0, False, cnt, cnt, True)                  # 2221
            max_z = max(self.Tri_bed_test_z)                                     # 2223
            min_z = min(self.Tri_bed_test_z)                                     # 2224
            dif_z = max_z - min_z                                                # 2225
            tm = datetime.datetime.now().strftime('[%Y/%m/%d-%H:%M:%S]')         # 2226
            z_str = str(self.Tri_bed_test_z).strip('[]')                         # 2227
            line = '{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(
                tm, self.tri_z_down_spd[0], str(now_pos[:-1]).strip('[]'), z_str,
                max_z, min_z, dif_z)                                             # 2228
            self.print_msg('WHY DEBUG', 'speed:{} pos:{} tri_z:{} Max:{:.3f} Min:{:.3f} (Max-Min):{:.3f}'.format(
                self.tri_z_down_spd[0], now_pos[:-1], self.Tri_bed_test_z, max_z, min_z, dif_z), True)  # 2229
            self.save_pres_file.writelines(line + '\n')                          # 2230
            self.Tri_bed_test_z = []                                             # 2231
        if self.save_pres_file is not None:                                      # 2233
            self.save_pres_file.close()                                          # 2234
            self.save_pres_file = None                                           # 2235
        self.tri_z_up_spd = tri_z_up_spd_bak                                     # 2236
        self.tri_z_down_spd = tri_z_down_spd_bak                                 # 2237
        self.bed_mesh.set_mesh(mesh)                                             # 2238

    cmd_SELF_CHECK_PRTOUCH_help = 'Self check the pres.'

    # L2241-2243. UNTESTED as a command (env_self_check itself is traced via
    # G28/AHZ). No G-code parameters; the return value is discarded.
    def cmd_SELF_CHECK_PRTOUCH(self, gcmd):
        self.env_self_check()                                                    # 2243

    cmd_TEST_PRTH_help = 'For Debug Cmd'

    # L2247. Empty body in the binary (returns None, no attribute access).
    def cmd_TEST_PRTH(self, gcmd):
        pass

    # lines 2273-2274. Not traced.
    def change_hot_min_temp(self, temp):
        self.hot_min_temp = temp
