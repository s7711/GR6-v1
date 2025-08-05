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
import numpy as np

class GadWheelspeed:
    def __init__(self):
        # Configuration
        self.lever_left_i = CFG["Imu2LeftWheel_i"]
        self.lever_right_i = CFG["Imu2RightWheel_i"]

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
        self.C_ib = ncomrx.RbnHPR(CFG['HPR_ib'])

        # Stats
        self.packets_sent = 0
        self.nextUpdateTime = time.monotonic()

        # On or off
        self.gad_type = 'GadVelocity'

    def update(self, machine_time, left_count, right_count):
        """
        Send wheelspeed aiding packets for both wheels.
        """
        try:
            if self.nrxs is None:
                logging.warning("GadWheelspeed: ncomrx object not set, skipping update.")
                return

            if self.nrx is None:
                try:
                    self.nrx = self.nrxs.nrx[CFG['InsIp']]['decoder']
                except:
                    pass # Not received yet
                    return

            gps_week, gps_seconds = self.nrx.mt2Gps(machine_time)
            left_velocity = left_count * CFG["WheelSf"]
            right_velocity = right_count * CFG["WheelSf"]

            if self.gad_type == 'GadSpeed':
                # Note: I have been unable to get GadSpeed to do anything
                # I cannot detect that it is functional using this code
                # Left wheel
                gw_left = oxts_sdk.GadSpeed(self.stream_id_left)
                if left_velocity >= 0:
                    gw_left.speed_fw_ms = left_velocity
                else:
                    gw_left.speed_bw_ms = -left_velocity
                gw_left.speed_ms_var = self.wheelspeed_var
                gw_left.time_gps = [gps_week, gps_seconds]
                gw_left.aiding_lever_arm_fixed = self.lever_left_i
                gw_left.aiding_lever_arm_var = self.lever_arm_var
                self.gh.send_packet(gw_left)
                self.packets_sent += 1

                # Right wheel
                gw_right = oxts_sdk.GadSpeed(self.stream_id_right)
                if right_velocity >= 0:
                    gw_right.speed_fw_ms = right_velocity
                else:
                    gw_right.speed_bw_ms = -right_velocity
                gw_right.speed_ms_var = self.wheelspeed_var
                gw_right.time_gps = [gps_week, gps_seconds]
                gw_right.aiding_lever_arm_fixed = self.lever_right_i
                gw_right.aiding_lever_arm_var = self.lever_arm_var
                self.gh.send_packet(gw_right)
                self.packets_sent += 1

            elif self.gad_type == 'GadVelocity':
                # Notes: This works, but isn't ideal. The lateral and vertical variances need to be
                # ~1.0, but we do not really know what they are. Putting them at 100.0 stops the
                # odom update from working/being useful.
                gv_left = oxts_sdk.GadVelocity(self.stream_id_left)
                V_i =  self.C_ib @ np.array([left_velocity, 0.0, 0.0])
                gv_left.vel_odom = V_i.tolist()
                Var_i = self.C_ib @ np.diag([self.wheelspeed_var, 1.0, 1.0]) @ self.C_ib.T
                gv_left.vel_odom_var = [Var_i[0][0], Var_i[1][1], Var_i[2][2], Var_i[0][1], Var_i[0][2], Var_i[1][2]]
                gv_left.time_gps = [gps_week, gps_seconds]
                gv_left.aiding_lever_arm_fixed = self.lever_left_i
                gv_left.aiding_lever_arm_var = self.lever_arm_var
                self.gh.send_packet(gv_left)
                self.packets_sent += 1

                gv_right = oxts_sdk.GadVelocity(self.stream_id_right)
                V_i = self.C_ib @ np.array([right_velocity, 0.0, 0.0])
                gv_right.vel_odom = V_i.tolist()
                # As above... Var_i = self.C_ib @ np.diag([self.wheelspeed_var, 100.0, 100.0]) @ self.C_ib.T
                gv_right.vel_odom_var = [Var_i[0][0], Var_i[1][1], Var_i[2][2], Var_i[0][1], Var_i[0][2], Var_i[1][2]]
                gv_right.time_gps = [gps_week, gps_seconds]
                gv_right.aiding_lever_arm_fixed = self.lever_right_i
                gv_right.aiding_lever_arm_var = self.lever_arm_var
                self.gh.send_packet(gv_right)
                self.packets_sent += 1

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
        except:
            logging.exception("[GadWheelSpeed, Update]: Exception during update")
    
    def user_command(self,message):
        try:
            args = message.split()
            if len(args) == 2 and args[0] == '<type':
                self.gad_type = args[1]
            elif len(args) == 2 and args[0] == '<var':
                self.wheelspeed_var = float(args[1])
            else:
                logging.warning("[gad_wheelspeed]: invalid command")
        except:
            logging.exception("[gad_wheelspeed]: invalid command")


