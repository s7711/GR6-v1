# path_follow.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Path following module. Implements a controller to get the robot to follow a path.
* Will send information to the web_server using ws.send("path-...",data)
* Receives navigation data from nrx.nav
* Sends motor signals using motor.send(Vl,Vr,Wp)
* Uses a thread to implement the control algorithm
* Sends the path to the website periodically

Set the path to follow using set_path(new_path) where new_path is a list of tuples:
  [(X0,Y0,S0,W0), (X1,Y1,S1,W1), (X2,Y2,S2,W2)...]
where XY are in metres, S (speed) is in metres/s and W is whether the water pump is on or off.

start() will start the set_path

stop() will stop path following immediately and set the motor speeds to zero (emergency)

From nrx['nav'] the path following uses:
  X = nrx.nav['MapLocalX'] (in metres)
  Y = nrx.nav['MapLocalY'] (in metres)
  H = nrx.nav['MapLocalHeading'] (in degrees)
"""

import threading
import time
import math
import logging
import json
import os

# Control parameters
LOOKAHEAD_DISTANCE = 0.4  # metres
HEADING_GAIN = 2.0        # gain for heading correction
CTE_GAIN = 0.6            # Cross track error gain
WHEEL_BASE = 0.42         # metres between wheel centers
MAX_MOTOR = 200
SCALE = 100 / 0.6         # motor units per m/s (based on 0.6 m/s ≈ 100)

# Abort parameters
MAX_LINE_DEPARTURE = 0.4  # metres
MAX_HEADING_CORRECTION = 70 * math.pi / 180.0
MAX_LOCALISATION_ERROR = 1.0 # metres RMS


class PathFollow:
    def __init__(self):
        self.path = None       # Holds a path, see above for format
        self.projection_index = 0 # Index into path for lookahead
        self.active = False    # True when actively contolling the robot
        self.integrated_error_sq = 0.0 # Summs all the errors during the path following
        self.ws = None         # Web server for sending information to web pages
        self.nrx = None        # Navigation data
        self.motor = None      # motor.send(Vl,Vr,Wp) for control signals
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        self.thread.start()
        self.path_control = {}
        self.path_dir = os.path.expanduser('~/paths')

    def set_path(self,new_path):
        """
        Sets a new path to follow. See above for path's format.
        """
        if self.active:
            logging.info("[Path following]: Cannot set a new path while following a path")
            return
        # Normalize path: ensure each tuple has 4 elements, padding with W=0 if needed
        new_path = [p if len(p) == 4 else (*p, 0) for p in new_path]

        # And check
        if not new_path or not all(len(p) == 4 for p in new_path):
            logging.warning("[Path following]: Path must be a list of (X, Y, S, W) tuples")
            return
        
        self.path = new_path
        self.projection_index = 0
        if self.ws is not None:
            self.ws.send("path-map",self.path)

    def start(self):
        if self.nrx is not None and self.motor is not None:        
            self.projection_index = 0
            x, y = self.nrx.nav['MapLocalX'], self.nrx.nav['MapLocalY']
            heading_deg = self.nrx.nav['MapLocalHeading']
            self._reset_projection_index(x,y,heading_deg,1,45)
            self.integrated_error_sq = 0.0
            self.active = True
            self.ws.send("path-control", {'State': 1})

    def stop(self):
        self.active = False
        self.motor.send(0,0,0)
        logging.info("[Path following]: Path following stopped")
        logging.info(f"[Path following]: Integrated error for path {self.integrated_error_sq**0.5:.2f}")
        self.ws.send("path-control", {'State': 0})

    def _loop(self):
        map_last_sent = time.time()
        control_last_sent = map_last_sent
        while True:
            try:
                if self.active:
                    self._update()
                now = time.time()
                if now - map_last_sent >= 10.0:
                    map_last_sent = now
                    if self.ws is not None:
                        self.ws.send("path-map", self.path)
                    # Send list of available path files
                    try:
                        filenames = [
                            f for f in os.listdir(self.path_dir)
                            if f.endswith('.path') and os.path.isfile(os.path.join(self.path_dir, f))
                        ]
                        self.ws.send("path-paths", filenames)
                    except Exception as e:
                        logging.warning(f"[PathFollow] Could not list path files: {e}")
                if now - control_last_sent >= 0.5:
                    control_last_sent = now
                    if self.ws is not None:
                        try:
                            self.path_control['State'] = 1 if self.active == True else 0
                            self.path_control['CurrentX'] = self.nrx.nav['MapLocalX']
                            self.path_control['CurrentY'] = self.nrx.nav['MapLocalY']
                            self.path_control['CurrentHeading'] = self.nrx.nav['MapLocalHeading']
                            ne, ee = self.nrx.status['NorthAcc'], self.nrx.status['EastAcc']
                            self.path_control['PositionAccuracy'] = math.hypot(ne, ee)
                        except: pass
                        self.ws.send("path-control", self.path_control)
                time.sleep(0.1)
            except Exception as e:
                logging.exception("[Path following]: Exception occurred in _loop")
                self.stop()

    def _update(self):
        x, y = self.nrx.nav['MapLocalX'], self.nrx.nav['MapLocalY']
        heading_deg = self.nrx.nav['MapLocalHeading']
        yaw_rad = math.radians(90.0 - heading_deg)

        # Find lookahead point
        result = self._find_lookahead_point(x, y, LOOKAHEAD_DISTANCE)
        if result is None:
            self.stop()
            return
        
        # Target point
        tx, ty, speed, proj_x, proj_y, cte = result
        dx, dy = (tx - x), (ty - y)
        target_angle = math.atan2(dy, dx)
        yaw_error = self._angle_diff(target_angle, yaw_rad)

        # Check if navigation is OK
        if abs(yaw_error) > MAX_HEADING_CORRECTION:
            logging.info(f"[Path following]:Aborting path, yaw error ({math.degrees(yaw_error)}) too large")
            logging.info(f"[Path following]:target_angle {math.degrees(target_angle)}")
            logging.info(f"[Path following]:yaw angle {math.degrees(yaw_rad)}")
            logging.info(f"[Path following]:Projection index {self.projection_index}")
            self.ws.send("path-control", {'Aborted': 1})
            self.stop()
            return
        
        dpx, dpy = (proj_x - x), (proj_y - y)
        pos_error = math.hypot(dpx, dpy)
        self.integrated_error_sq += pos_error **2
        if pos_error > MAX_LINE_DEPARTURE:
            logging.info(f"[Path following]:Aborting path, position error too large ({dpx:.2f} {dpy:.2f})")
            logging.info(f"[Path following]: (x,y) = ({x:.2f},{y:.2f}), (proj_x,proj_y) = {proj_x:.2f},{proj_y:.2f}")
            logging.info(f"[Path following]: Projection index = {self.projection_index}")
            logging.info(f"[Path following]: Point at projection index = ({self.path[self.projection_index]})")
            self.ws.send("path-control", {'Aborted': 1})
            self.stop()
            return
        
        ne, ee = self.nrx.status['NorthAcc'], self.nrx.status['EastAcc']
        pos_acc = math.hypot(ne, ee)
        if pos_acc > MAX_LOCALISATION_ERROR:
            logging.info("[Path following]:Aborting path, position accuracy too low")
            self.ws.send("path-control", {'Aborted': 1})
            self.stop()
            return

        # Compute wheel speeds
        forward = speed
        turn = HEADING_GAIN * yaw_error + CTE_GAIN * cte # Not tuned yet, or even verified the sign of cte

        Vl = forward - (turn * WHEEL_BASE / 2)
        Vr = forward + (turn * WHEEL_BASE / 2)

        # Convert to motor units
        ml = SCALE * Vl
        mr = SCALE * Vr

        # Find the maximum absolute motor value
        max_val = max(abs(ml), abs(mr), MAX_MOTOR)

        # Scale down if needed
        if max_val > MAX_MOTOR:
            scale_factor = MAX_MOTOR / max(abs(ml), abs(mr))
            ml *= scale_factor
            mr *= scale_factor

        # Convert to integers
        ml = int(ml)
        mr = int(mr)

        # Find the water pump value (finds it for the look-ahead, not the current position)
        point = self.path[self.projection_index]
        w = point[3] if len(point) > 3 else 0
        
        self.motor.send(ml, mr, w)

        self.path_control.update({
            'YawError': yaw_error,
            'TargetX': tx,
            'TargetY': ty,
            'Forward': forward,
            'Turn': turn,
            'LeftMotor': ml,
            'RightMotor': mr,
            'LookaheadDistance': LOOKAHEAD_DISTANCE,
            'PositionError': pos_error,
            'PathIndex': self.projection_index,
            'PathLength': len(self.path),
            'IntegratedError': self.integrated_error_sq**0.5,
            'Aborted': 0
        })

    def _angle_diff(self, a, b):
        """Returns the smallest difference between two angles (in radians)"""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d
    
    def _find_lookahead_point(self, robot_x, robot_y, lookahead_distance):
        if not self.path or len(self.path) < 2:
            return None

        best_index = None
        best_proj = None
        best_dist = float('inf')
        i = self.projection_index
        last_dist = float('inf')

        while i < len(self.path) - 1:
            x1, y1, _, _ = self.path[i]
            x2, y2, _, _ = self.path[i + 1]
            dx, dy = (x2 - x1), (y2 - y1)
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                i += 1
                continue

            # Project robot onto segment
            t = ((robot_x - x1) * dx + (robot_y - y1) * dy) / length_sq
            t = max(0, min(1, t))
            proj_x, proj_y = (x1 + t * dx), (y1 + t * dy)
            dist = math.hypot(proj_x - robot_x, proj_y - robot_y)

            if dist < best_dist:
                best_dist = dist
                best_proj = (proj_x, proj_y, t)
                best_index = i
            elif dist > last_dist:
                break  # distance is increasing, stop search

            last_dist = dist
            i += 1

        if best_proj is None:
            return None

        self.projection_index = best_index

        # Step 2: walk forward from projection to find lookahead point
        remaining = lookahead_distance
        x1, y1, s1, _ = self.path[best_index]     # starting point in path
        x2, y2, _, _ = self.path[best_index + 1]  # next point in path
        dx, dy = (x2 - x1), (y2 - y1)          
        seg_len = math.hypot(dx, dy)           # length of this segment
        seg_remain = (1 - best_proj[2]) * seg_len # remaining part in this segment

        # Compute cross track error
        rx = robot_x - best_proj[0]
        ry = robot_y - best_proj[1]
        cte = (dx * ry - dy * rx) / seg_len

        if seg_remain >= remaining:
            # then lookahead is in this segment
            ratio = (best_proj[2] * seg_len + remaining) / seg_len
            lx = x1 + ratio * dx
            ly = y1 + ratio * dy
            return (lx, ly, s1, best_proj[0], best_proj[1], cte) # lookahead point, speed, closest point on the path, cross track error

        remaining -= seg_remain
        i = best_index + 1

        while i < len(self.path) - 1:
            x1, y1, s1, _ = self.path[i]
            x2, y2, _, _ = self.path[i + 1]
            seg_len = math.hypot(x2 - x1, y2 - y1)
            if seg_len >= remaining:
                ratio = remaining / seg_len
                lx = x1 + ratio * (x2 - x1)
                ly = y1 + ratio * (y2 - y1)
                return (lx, ly, s1, best_proj[0], best_proj[1], cte) # lookahead point, speed, closest point on the path, cross track error
            remaining -= seg_len
            i += 1

        return None # Lookahead beyond path
    
    def _reset_projection_index(self, robot_x, robot_y, robot_heading_deg, max_search_distance=1.0, heading_tolerance_deg=45):
        robot_heading = math.radians(robot_heading_deg)
        heading_tolerance = math.radians(heading_tolerance_deg)

        logging.info(f"[Path following]: Finding initial projection point for X{robot_x:.2f}, Y{robot_y:.2f}, H{robot_heading_deg:.1f}")

        for i in range(len(self.path) - 1):
            x1, y1, _, _ = self.path[i]
            x2, y2, _, _ = self.path[i + 1]
            dx, dy = x2 - x1, y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq == 0:
                continue

            # Project robot onto segment
            t = ((robot_x - x1) * dx + (robot_y - y1) * dy) / seg_len_sq
            t = max(0, min(1, t))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dist = math.hypot(proj_x - robot_x, proj_y - robot_y)

            if dist > max_search_distance:
                continue

            # Check heading alignment
            path_heading = (math.pi / 2 - math.atan2(dy, dx)) % (2 * math.pi)
            heading_diff = abs(self._angle_diff(path_heading, robot_heading))
            logging.info(f"[Path following]: Index {i}, Path heading: {path_heading:.1f}, Robot misalignment: {heading_diff:.1f}")
            if heading_diff > heading_tolerance:
                continue

            # Found a valid segment
            self.projection_index = i
            return True

        return False  # No valid segment found


    def user_command(self, message):
        try:
            args = message.split()

            if message.startswith('>set-path '):
                path_json = message[len('>set-path '):]
                path = json.loads(path_json)
                self.set_path(path)
                logging.info(f"[PathFollow] Received new path with {len(path)} points.")

            elif message.startswith('>save-path '):
                filename = message[len('>load-path '):].strip()
                if not filename.endswith('.path'): filename += '.path'
                os.makedirs(self.path_dir, exist_ok=True)
                full_path = os.path.join(self.path_dir, filename)
                with open(full_path, 'w') as f:
                    for x, y, speed, wp in self.path:
                        f.write(f"G1 X{x:.3f} Y{y:.3f} F{speed} W{wp}\n") # g-code format
                logging.info(f"[PathFollow] Saved path to {full_path}")

            elif message.startswith('>load-path '):
                filename = message[len('>load-path '):].strip()
                if not filename.endswith('.path'): filename += '.path'
                os.makedirs(self.path_dir, exist_ok=True)
                full_path = os.path.join(self.path_dir, filename)
                self.check_for_old_format(full_path) # Will re-write in new format
                path = self.load_gcode_path(full_path)
                self.set_path(path)
                logging.info(f"[PathFollow] Loaded path from {full_path} with {len(path)} points.")
                if self.ws is not None:
                    self.ws.send("path-map", self.path)

            elif message == '>start-path':
                self.start()
                logging.info("[PathFollow] Starting path following.")

            elif message == '>stop-path':
                self.stop()
                logging.info("[PathFollow] Stopping path following.")
            
            elif message.startswith('>LOOKAHEAD_DISTANCE'):
                LOOKAHEAD_DISTANCE = float(args[1])
            elif message.startswith('>HEADING_GAIN'):
                HEADING_GAIN = float(args[1])
            elif message.startswith('>CTE_GAIN'):
                CTE_GAIN = float(args[1])
        except Exception as e:
            logging.exception(f"[PathFollow] Error handling command '{message}': {e}")

    def check_for_old_format(self, full_path):
        try:
            with open(full_path, 'r') as f:
                first_char = f.read(1)
                f.seek(0)  # Reset file pointer

                if first_char in ['[', '{']:
                    path = json.load(f)

                    # Rewrite in G-code format
                    logging.info(f"[path following]]: Re-writing old json path to gcode format for file {full_path}")
                    with open(full_path, 'w') as out:
                        for x, y, speed in path:
                            out.write(f"G1 X{x:.3f} Y{y:.3f} F{speed}\n")
        except:
            logging.exception("[path_follow]: Error checking or writing for old path file format")


    def load_gcode_path(self, full_path):
        try:
            path = []

            with open(full_path, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line.startswith("G1"):
                        continue

                    x = y = f = None
                    w = 0 # May not be present in gcode file, so default to 0 (off)
                    tokens = line.split()

                    for token in tokens[1:]:  # Skip 'G1'
                        if token.startswith("X"):
                            try: x = float(token[1:])
                            except ValueError: logging.warning(f"Line {line_num}: Invalid X value '{token}'")
                        elif token.startswith("Y"):
                            try: y = float(token[1:])
                            except ValueError: logging.warning(f"Line {line_num}: Invalid Y value '{token}'")
                        elif token.startswith("F"):
                            try: f = float(token[1:])
                            except ValueError: logging.warning(f"Line {line_num}: Invalid F value '{token}'")
                        elif token.startswith("W"):
                            try: w = float(token[1:])
                            except ValueError: logging.warning(f"Line {line_num}: Invalid W value '{token}'")

                    if x is not None and y is not None and f is not None:
                        path.append((x, y, f, w))
                    else:
                        logging.warning(f"Line {line_num}: Missing required fields in '{line}'")
                        return [] # Brutal, but safer
            return path
        except:
            logging.exception("[path_following]: Exception while loading path")
        return []
