# sys_stat.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Routines that collect information about the system and send to a web_server
"""

import threading
import subprocess
import psutil
import time
import os

class sysStat:
    def __init__(self):
        self.status = {
            "WifiSigStrength": None,  # dBm
            "CpuUsage": None,         # Percent
            "CpuTemp": None           # Degrees Celsius
        }
        self.ws = None
        self._stop_flag = threading.Event()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _get_wifi_strength(self):
        try:
            output = subprocess.check_output(["iwconfig"], stderr=subprocess.DEVNULL).decode()
            for line in output.split("\n"):
                if "Signal level" in line:
                    parts = line.strip().split("Signal level=")
                    if len(parts) > 1:
                        value = parts[1].split(" ")[0]
                        return int(value.replace("dBm", ""))
        except Exception:
            pass
        return None

    def _get_cpu_temp(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                millidegree = int(f.read().strip())
                return round(millidegree / 1000.0, 1)
        except Exception:
            return None

    def _update_loop(self):
        while not self._stop_flag.is_set():
            dbm = self._get_wifi_strength()
            self.status["WifiSigStrengthDbm"] = dbm
            self.status["WifiSigStrength"] = self._scale_wifi_strength(dbm)
            self.status["CpuUsage"] = psutil.cpu_percent(interval=1.0)
            self.status["CpuTemp"] = self._get_cpu_temp()
            #time.sleep(0.5) #psutil will limit the update rate
            if self.ws is not None:
                self.ws.send("SysStatus",self.status)

    def stop(self):
        self._stop_flag.set()
        self._thread.join()

    def _scale_wifi_strength(self, dbm):
        if dbm is None:
            return 0  # No signal detected
        if dbm >= -50:
            return 5
        elif dbm >= -65:
            return 4
        elif dbm >= -75:
            return 3
        elif dbm >= -85:
            return 2
        else:
            return 1

