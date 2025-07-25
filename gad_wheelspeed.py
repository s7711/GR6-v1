# gad_wheelspeed.py
# Licensed under the MIT License – see LICENSE file for details.

"""
Module to send wheelspeed aiding data to the OxTS INS using the GAD SDK.
Designed for use with quadrature encoder counts and monotonic timestamps.
"""

import oxts_sdk
import logging
from config import CFG
import time
import datetime
import ncomrx

class GadWheelspeed:
    def __init__(self):
        # Configuration
        self.lever_left = CFG["WheelPosLeft"]
        self.lever_right = CFG["WheelPosRight"]

        # Fixed parameters
        self.stream_id_left = 133
        self.stream_id_right = 134
        self.wheelspeed_var = 0.01  # m²/s²
        self.lever_arm_var = [0.001, 0.001, 0.001] # ~3cm

        # External interfaces
        self.nrxs = None # All the received INSs
        self.nrx = None  # The configured INS from CFG["InsIp"]
        self.ws = None

        # SDK handler
        self.gh = oxts_sdk.GadHandler()
        self.gh.set_encoder_to_bin()
        self.gh.set_output_mode_to_udp(CFG["InsIp"])

        # Stats
        self.packets_sent = 0
        self.nextUpdateTime = time.monotonic()

    def update(self, machine_time, left_count, right_count):
        """
        Send wheelspeed aiding packets for both wheels.
        """

        if self.nrxs is None:
            logging.warning("GadWheelspeed: ncomrx object not set, skipping update.")
            return

        if self.nrx is None:
            try:
                self.nrx = self.nrxs.nrx[CFG['InsIp']]['decoder']
            except:
                pass # Not received yet
                return

        try:
            gps_week, gps_seconds = self.nrx.mt2Gps(machine_time)
        except Exception as e:
            logging.warning(f"GadWheelspeed: GPS time conversion failed: {e}")
            return

        # Left wheel
        left_velocity = left_count * CFG["WheelSf"]
        gw_left = oxts_sdk.GadSpeed(self.stream_id_left)
        if left_velocity >= 0:
            gw_left.speed_bw_ms = 0.0
            gw_left.speed_fw_ms = left_velocity
        else:
            gw_left.speed_fw_ms = 0.0
            gw_left.speed_bw_ms = -left_velocity
        gw_left.speed_ms_var = self.wheelspeed_var
        gw_left.time_gps = [gps_week, gps_seconds]
        gw_left.aiding_lever_arm_fixed = self.lever_left
        gw_left.aiding_lever_arm_var = self.lever_arm_var

        try:
            self.gh.send_packet(gw_left)
            self.packets_sent += 1
        except Exception as e:
            logging.warning(f"GadWheelspeed: Failed to send left packet: {e}")

        # Right wheel
        right_velocity = right_count * CFG["WheelSf"]
        gw_right = oxts_sdk.GadSpeed(self.stream_id_right)
        if right_velocity >= 0:
            gw_right.speed_bw_ms = 0.0
            gw_right.speed_fw_ms = right_velocity
        else:
            gw_right.speed_fw_ms = 0.0
            gw_right.speed_bw_ms = -right_velocity
        gw_right.speed_ms_var = self.wheelspeed_var
        gw_right.time_gps = [gps_week, gps_seconds]
        gw_right.aiding_lever_arm_fixed = self.lever_right
        gw_right.aiding_lever_arm_var = self.lever_arm_var

        try:
            self.gh.send_packet(gw_right)
            self.packets_sent += 1
        except Exception as e:
            logging.warning(f"GadWheelspeed: Failed to send right packet: {e}")

        # Telemetry
        t = time.monotonic()
        if self.ws is not None and self.nextUpdateTime < t:
            self.nextUpdateTime = t + 0.5  # Update
            self.ws.send("gad_wheelspeed", {
                "GwsLeftVelocity": left_velocity,
                "GwsRightVelocity": right_velocity,
                "GwsLeftCount": left_count,
                "GwsRightCount": right_count,
                "GwsGpsTime" : ncomrx.GPS_STARTTIME + \
                    datetime.timedelta( minutes=gps_week*10080, seconds=gps_seconds ), 
                "GwsGpsWeek": gps_week,
                "GwsGpsSeconds": gps_seconds,
                "GwsPacketsSent": self.packets_sent
            })

