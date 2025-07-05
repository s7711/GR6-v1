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
import logging

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baud=9600, ws=None):
        self.arduino = serial.Serial(port, baudrate=baud, timeout=0.5)
        self.ws = ws
        self.motor_state = {}
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self.running:
            if self.arduino.in_waiting:
                try:
                    line = self.arduino.readline().decode('utf-8')
                    self._decode_line(line)
                    if self.ws:
                        self.ws.send("motor", self.motor_state)
                except Exception as e:
                    logging.exception("[Serial Error]", e)

    def _decode_line(self, line):
        if line.startswith("SP"):
            try:
                parts = line.strip().split()
                self.motor_state.update({
                    'LM_setpoint': int(parts[1]),
                    'RM_setpoint': int(parts[2]),
                    'LM_position': int(parts[4]),
                    'RM_position': int(parts[5]),
                    'LM_error': int(parts[7]),
                    'RM_error': int(parts[8]),
                    'LM_pwm': int(parts[10]),
                    'RM_pwm': int(parts[11])
                })
            except Exception as e:
                logging.exception("[Decode Error]", e)
        else:
            logging.info("From arduino: " + line)

    def user_command(self, message):
        if message.startswith("&"):
            cmd = message[1:]
            self.arduino.write((cmd + "\n").encode())
            logging.info("[Arduino]:" + cmd)
    
    def send(self,lm,rm):
        self.arduino.write((f"SV {int(lm)} {int(rm)}\n").encode())

    def shutdown(self):
        self.running = False
        self.thread.join()
        self.arduino.close()
