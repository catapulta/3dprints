# Let a print cancel stop an in-flight homing move.
#
# Moonraker's cancel arrives on the "pause_resume/cancel" webhook, whose
# handler queues CANCEL_PRINT behind the gcode mutex. G28 holds that mutex
# until homing finishes, so the cancel normally waits for the whole homing
# sequence (and the rest of the running macro).
#
# This module wraps that webhook. While a cancel is pending it fires the
# active homing move's trsync with REASON_HOST_REQUEST, the same MCU path an
# endstop hit takes: steppers stop, homing_move_end handlers (TMC
# stallguard restore etc.) run, and G28 fails with a command error. The
# error aborts the running macro / virtual_sdcard print, then the queued
# CANCEL_PRINT runs as usual. Any homing move that starts while the cancel
# is still pending is stopped the same way.
#
# Probing with prtouch_v2 runs inside Creality's closed run_G29_Z() and
# never fires the homing_move events, so an in-flight probe can't be
# interrupted. Instead homing.probing_move is wrapped: while a cancel is
# pending, the next probe point raises a command error before it starts, so
# BED_MESH_CALIBRATE (KAMP) stops after the point in progress. probe.py's
# gcode:command_error handler then ends the multi-probe session.
#
# Config: add an empty [homing_abort] section.
import logging

RETRY_INTERVAL = 0.05


class HomingAbort:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.cancel_pending = False
        self.active_hmove = None
        self.orig_cancel = None
        self.retry_timer = None
        self.printer.register_event_handler(
            "homing:homing_move_begin", self._handle_move_begin)
        self.printer.register_event_handler(
            "homing:homing_move_end", self._handle_move_end)
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

    def _handle_connect(self):
        webhooks = self.printer.lookup_object('webhooks')
        endpoints = webhooks._endpoints
        self.orig_cancel = endpoints.get("pause_resume/cancel")
        if self.orig_cancel is None:
            logging.info("homing_abort: no pause_resume/cancel endpoint")
            return
        endpoints["pause_resume/cancel"] = self._handle_cancel_request
        phoming = self.printer.lookup_object('homing', None)
        if phoming is not None:
            orig_probing_move = phoming.probing_move
            def probing_move(mcu_probe, pos, speed):
                if self.cancel_pending:
                    raise self.printer.command_error(
                        "Probing aborted by cancel request")
                return orig_probing_move(mcu_probe, pos, speed)
            phoming.probing_move = probing_move

    def _handle_cancel_request(self, web_request):
        self.cancel_pending = True
        try:
            if self.active_hmove is not None:
                self._start_abort(self.active_hmove)
            # Blocks until CANCEL_PRINT has run.
            self.orig_cancel(web_request)
        finally:
            self.cancel_pending = False

    def _handle_move_begin(self, hmove):
        self.active_hmove = hmove
        if self.cancel_pending:
            self._start_abort(hmove)

    def _handle_move_end(self, hmove):
        if self.active_hmove is hmove:
            self.active_hmove = None
        if self.retry_timer is not None:
            self.reactor.unregister_timer(self.retry_timer)
            self.retry_timer = None

    def _start_abort(self, hmove):
        # homing_move_begin fires before the endstops are armed, so keep
        # firing until the move ends; a trigger on an unarmed trsync is a
        # no-op on the MCU.
        if self.retry_timer is not None:
            return
        self.gcode.respond_info("Homing aborted by cancel request")
        def retry(eventtime):
            if self.active_hmove is not hmove:
                return self.reactor.NEVER
            self._trigger(hmove)
            return eventtime + RETRY_INTERVAL
        self.retry_timer = self.reactor.register_timer(
            retry, self.reactor.NOW)

    def _trigger(self, hmove):
        try:
            for mcu_endstop in hmove.get_mcu_endstops():
                for trsync in getattr(mcu_endstop, '_trsyncs', []):
                    trsync._trsync_trigger_cmd.send(
                        [trsync._oid, trsync.REASON_HOST_REQUEST])
        except Exception:
            logging.exception("homing_abort: trsync trigger failed")

def load_config(config):
    return HomingAbort(config)
