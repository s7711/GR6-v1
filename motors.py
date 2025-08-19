# motors.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Used for sending and receiving commands from the arduino
which controls the motors

Currently expecting arduino code "GR6_motor_250630"

Notes on the motor setup:
* A commanded speed of 100 gives about 160 steps/s
* The encoder and wheels have about 250 steps/metre
* So SV100 is about 0.6 m/s
* There's no control algorithm, so load will change the speed a lot
"""

import serial
import threading
import time
import logging

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baud=115200, ws=None):
        self.arduino = serial.Serial(port, baudrate=baud, timeout=0.5)
        self.ws = ws
        self.gad_wheelspeed = None
        self.last_EN = None
        self.motor_state = {}
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self.running:
            if self.arduino.in_waiting:
                try:
                    line = self.arduino.readline().decode('utf-8')
                    send = self._decode_line(line)
                    if self.ws and send:
                        self.ws.send("motor", self.motor_state)
                except Exception as e:
                    logging.exception("[Serial Error]", e)

    def _decode_line(self, line):
        """
        Parse one line of serial telemetry and update self.motor_state.
        Expects labels:
        EN  - encoder positions (raw ints)
        SV  - set-velocities (*100 int to float)
        FV  - filtered velocities (*100 int to float)
        ER  - error signals (*100 int to float)
        EI  - integral errors (*100 int to float)
        MO  - motor outputs (raw floats)
        WP  - waypoint/WaterPump state (int)
        GO  - control enabled flag (0/1)
        Kp  - proportional gain (*100 int to float)
        Ki  - integral gain (*100 int to float)
        Kv  - velocity feed-forward (*100 int to float)
        Kd  - acceleration gain (*100 int to float)
        Ka  - acceleration feed-forward (*100 int to float)
        Kb  - bias term (*100 int to float)
        Db  - deadband (raw int)
        Mi  - max integral (raw int)
        Mj  - min integral (raw int) - should be negative
        Id  - integral decay (*100 int to float)
        """
        try:
            parts = line.strip().split()
            if not parts:
                return False

            tag = parts[0]
            # helper lambdas for conversion
            to_f100 = lambda x: int(x) / 100.0
            to_i    = lambda x: int(x)
            to_b    = lambda x: bool(int(x))

            if tag == "EN" and len(parts) >= 3:
                # Both GAD and Encoder for website
                EN = (to_i(parts[1]), to_i(parts[2]))
                if self.last_EN is None:
                    self.last_EN = EN
                else:
                    # Calculate the change in encoder position
                    delta_EN = (EN[0] - self.last_EN[0], EN[1] - self.last_EN[1])
                    self.motor_state.update({
                        "LM_delta": delta_EN[0],
                        "RM_delta": delta_EN[1]
                    })
                    self.last_EN = EN
                    if self.gad_wheelspeed:
                        self.gad_wheelspeed.update(time.monotonic(), delta_EN[0], delta_EN[1])

                self.motor_state.update({
                    "LM_position": EN[0],
                    "RM_position": EN[1]
                })
                return True # Will send to the web page(s)

            elif tag == "SV" and len(parts) >= 3:
                self.motor_state.update({
                    "LM_setvel": to_f100(parts[1]),
                    "RM_setvel": to_f100(parts[2])
                })

            elif tag == "FV" and len(parts) >= 3:
                self.motor_state.update({
                    "filtLM_vel": to_f100(parts[1]),
                    "filtRM_vel": to_f100(parts[2])
                })

            elif tag == "EA" and len(parts) >= 3:
                self.motor_state.update({
                    "filtLM_dErr": to_f100(parts[1]),
                    "filtRM_dErr": to_f100(parts[2])
                })

            elif tag == "ER" and len(parts) >= 3:
                self.motor_state.update({
                    "errLM": to_f100(parts[1]),
                    "errRM": to_f100(parts[2])
                })

            elif tag == "EI" and len(parts) >= 3:
                self.motor_state.update({
                    "integralLM": to_f100(parts[1]),
                    "integralRM": to_f100(parts[2])
                })

            elif tag == "MO" and len(parts) >= 3:
                self.motor_state.update({
                    "outLM": float(parts[1]),
                    "outRM": float(parts[2])
                })

            elif tag == "WP" and len(parts) >= 2:
                self.motor_state.update({
                    "WaterPump": to_i(parts[1])
                })

            elif tag == "GO" and len(parts) >= 2:
                self.motor_state.update({
                    "ctrlEnabled": to_b(parts[1])
                })

            elif tag == "Kp" and len(parts) >= 3:
                self.motor_state["LM_Kp"] = to_f100(parts[1])
                self.motor_state["RM_Kp"] = to_f100(parts[2])

            elif tag == "Ki" and len(parts) >= 3:
                self.motor_state["LM_Ki"] = to_f100(parts[1])
                self.motor_state["RM_Ki"] = to_f100(parts[2])

            elif tag == "Kd" and len(parts) >= 3:
                self.motor_state["LM_Kd"] = to_f100(parts[1])
                self.motor_state["RM_Kd"] = to_f100(parts[2])

            elif tag == "Kf" and len(parts) >= 3:
                self.motor_state["LM_Kf"] = to_f100(parts[1])
                self.motor_state["RM_Kf"] = to_f100(parts[2])

            elif tag == "Ka" and len(parts) >= 3:
                self.motor_state["LM_Ka"] = to_f100(parts[1])
                self.motor_state["RM_Ka"] = to_f100(parts[2])

            elif tag == "Kb" and len(parts) >= 3:
                self.motor_state["LM_Kb"] = to_f100(parts[1])
                self.motor_state["RM_Kb"] = to_f100(parts[2])

            elif tag == "Db" and len(parts) >= 3:
                self.motor_state["LM_Db"] = to_i(parts[1])
                self.motor_state["RM_Db"] = to_i(parts[2])

            elif tag == "Mi" and len(parts) >= 3:
                self.motor_state["LM_Mi"] = to_i(parts[1])
                self.motor_state["RM_Mi"] = to_i(parts[2])

            elif tag == "Mj" and len(parts) >= 3:
                self.motor_state["LM_Mj"] = to_i(parts[1])
                self.motor_state["RM_Mj"] = to_i(parts[2])

            elif tag == "Id" and len(parts) >= 3:
                self.motor_state["LM_Id"] = to_f100(parts[1])
                self.motor_state["RM_Id"] = to_f100(parts[2])

            elif tag == "Am" and len(parts) >= 3:
                self.motor_state["LM_Am"] = float(parts[1])
                self.motor_state["RM_Am"] = float(parts[2])
            
            elif tag.startswith("U") and len(parts) == 2:
                self.motor_state[tag] = float(parts[1])*0.001 # Convert to mm
            
            elif tag == "Version" and len(parts) >= 2:
                self.motor_state["MotorVersion"] = parts[1]

            # else: unrecognized tag—ignore or log if you wish
        except Exception as e:
            logging.exception("[Decode Error] %s", e)

        return False

    def user_command(self, message):
        if message.startswith("&"):
            cmd = message[1:]
            self.arduino.write((cmd + "\n").encode())
            logging.info("[Arduino]:" + cmd)
    
    def send(self,lm,rm,w):
        self.arduino.write((f"SV {int(lm)} {int(rm)}\n").encode())
        self.arduino.write((f"WP {int(w)}\n").encode())

    def shutdown(self):
        self.running = False
        self.thread.join()
        self.arduino.close()
