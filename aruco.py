# aruco.py
# Licensed under the MIT License – see LICENSE file for details.

"""
aruco.py
Looks for aruco markers, computes the distance and angles.

Reads marker map.csv file, csv format. The map should
have the following columns:
AmId        - Matching the Aruco dictionary
AmSize      - Metres, currently ignored
AmX         - Metres from origin
AmY         - Metres from origin
AmZ         - Metres from origin
AmHeading   - Decimal degrees
AmPitch     - Decimal degrees
AmRoll      - Decimal degrees

Notes on the rotations. Help from web pages:
  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
The camera co-ordinates are on page:
  https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html

  
There is a co-ordinate mess going on in this file, so here are the
conventions that are bring used (hopefully consistently).

Note: Originally I was using V_a = R_ba * V_b. But that doesn't seem to
be as common as V_a = C_ab * V_b. (The ba and ab are swapped). I am
swapping to the more conventional notation, and using "C_" instead of "R_"
so I know where things have been fixed. This will cause confusion!

The base frame is the NED (north, east down) frame, normally known
as the "navigation" frame:
  X_n - north
  Y_n - east
  Z_n - down
These are from the origin, at CFG['AmBaseLLA'].

The vehicle frame, known in navigation as the "body" frame has:
  X_b - forward
  Y_b - right
  Z_b - down

The navigation frame is related to the body (vehicle) frame through
heading, pitch and roll:
  V_n = C_nb(HPR) * V_b
where C_nb(HPR) is defined in the RT user manual as:
  C_nb(HPR) = C_Heading * C_Pitch * C_Roll
i.e. C_Heading is the rotation about the Z_n axis, etc.

The camera co-ordinates (_C) are very different, with:
  X_C right (viewed from behind the camera)
  Y_C down
  Z_C along the camera axis towards the scene

It's much easier to use a camera co-ordinate system that allows us to
express the camera's orientation in the vehicle using Heading, Pitch and Roll
Euler angle rotations. This co-ordinate system will be "_c" and will be:
  X_c - forward out of the camera, towards the scene
  Y_c - right (viewed from behind the camera)
  Z_c - down in the image

The marker co-ordinates used by aruco (_M) are:
  X_M right (from the front of the marker)
  Y_M up
  Z_M towards the camera (looking from behind the marker)

The marker co-ordinates used in the file (_m) are:
  X_m out the back of the marker
  Y_m to the right of the marker
  Z_m down
(These make it easy for the HmPmRm work correctly.)
  
The definition of the marker angles, HmPmRm (marker's heading, pitch, roll
in the map file), are in the OxTS convention. For a marker with Hm=0, Pm=0 and Rm=0 then:
  X_M (marker's X) points east    X_m points north
  Y_M (marker's Y) points up      Y_m points east
  Z_M (marker's Z) points south   Z_m points down
And if Hm=90 then X_m points east, etc.

Aruco estimates the pose:
"marker coordinate system to the camera coordinate system"
written as C_CM (or rotation from marker to camera)

The full chain of rotations is such that:
  C_nb = C_nm * C_mM * C_MC * C_Cc * C_cb
where:
  C_nm - navigation to marker frame, in the map file as HmPmRm
  C_mM - definitions, see axes above
  C_MC - (C_CM = Rodrigues()).T from aruco code
  C_Cc - definitions, see axes above
  C_cb - camera mount in the vehicle
"""

import threading
import collections
import math
import cv2
import numpy as np
import csv
import os
import traceback
import logging
import time

import ncomrx # for useful functions

class Aruco:
    """
    Looks for Aruco markers, computes the vector and angles ready to
    send to an OxTS INS
    """
    def __init__(self, mapfile='map.csv', C_cb=None, Dxc_b=None):
        # Queue for images from the camera, modified so the marker(s)
        # are included on the image
        # This application doesn't make much use of the queue, but
        # it would be useful for, say, comparing 5 images to see
        # if they are stationary or moving
        self.cam_frames = collections.deque(maxlen=5)
        self.cam_condition = threading.Condition()

        # Load/create parameters for aruco marker calculations
        homeDir = os.path.expanduser('~')
        fs = cv2.FileStorage(os.path.join(homeDir,'.gad_camcal.yaml'), cv2.FILE_STORAGE_READ)
        try:
            r = tuple(fs.getNode("resolution").mat())
            self.calibrationResolution = (r[1][0],r[0][0]) # OpenCV does resolutions backwards!
        except:
            logging.exception("Camera calibration resolution unknown")
            self.calibrationResolution = None
        self.mtx = fs.getNode("camera_matrix").mat()
        self.dist = fs.getNode("distortion_coefficients").mat()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Method of stopping the processing and terminating the thread
        self.go = True

        self.show_rejected = False
        
        # Load the mapfile
        # markers is a dictionary of the markers where the key is the
        # marker id and the value is a dictionary of the marker
        # information from the CSV file, see comment at the top of the
        # file
        # For example { 201 : { AmId:201, AmSize:0.1, AmN: 30.0, AmE:53.0, ... },
        #                19 : { AmId:19,  AmSize:0.2, AmN: 35.0, AmE:55.0, ... } }
        self.markers = {}
        try:
            with open(mapfile, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    self._numberify(row)
                    self.markers.update({row['AmId']:row})
        except Exception as e:
            logging.exception(traceback.format_exc())

        # Rotation matricies so the marker rotation can be expressed as HPR
        self.C_mM = np.array([[ 0.0,  0.0, -1.0],
                              [ 1.0,  0.0,  0.0],
                              [ 0.0, -1.0,  0.0]])
        
        # Rotation to express vectors from the opencv camera co-ordinate system
        # in a frame more consistent with OXTS, so HPR-like rotations can be expressed
        self.C_Cc = np.array([[0.0,1.0,0.0],
                              [0.0,0.0,1.0],
                              [1.0,0.0,0.0]])
        self.C_cb = np.eye(3) if C_cb is None else C_cb
        self.Dxc_b = np.zeros(3) if Dxc_b is None else Dxc_b
        
        # Unmapped markers seen by the camera
        # Put in the map format
        # For example { 201 : { AmId:201, AmSize:0.1, AmN: 20.0, AmE:40.0, ... },
        #                19 : { AmId:19,  AmSize:0.1, AmN: 30.0, AmE:30.0, ... } }
        # Will have to assume a size. TODO: resolve a size somehow.
        self.unmappedMarkers = {}


    def _numberify(self, d):
        """
        Converts dictionary object from text to numbers
        """
        for k,v in d.items():
            # Try and convert the value to an integer
            try:
                i = int(v)
                d[k] = i
            except:
                # Didn't work, so try and convert to a float
                try:
                    f = float(v)
                    d[k] = f
                except:
                    pass # It's a string and it remains a string




    def _C_nb(self,H,P,R):
        """
        Computes the direction cosine matrix rotation from body to navigation
        frame from Heading, Pitch and Roll (in degrees), such that:
        V_n = _C_nb(HPR) * V_b
        Where V_n is the vector in the navigation frame (North, East, Down)
        and V_b is the vector in the body frame (X-forward, Y-right, Z-down)
        See RT User Manual page 101 for details:
        https://www.oxts.com/wp-content/uploads/2020/03/rtman-200302.pdf
        (page 101 in version 200220)
        """
        H = H * math.pi/180.0
        P = P * math.pi/180.0
        R = R * math.pi/180.0
        # Rotation from body to navigation frame using Heading, Pitch, Roll in radians
        C_h = np.array([[math.cos(H), -math.sin(H), 0.0],
                        [math.sin(H),  math.cos(H), 0.0],
                        [0.0,          0.0,         1.0] ])
        C_p = np.array([[ math.cos(P), 0.0, math.sin(P)],
                        [ 0.0,         1.0,         0.0],
                        [-math.sin(P), 0.0, math.cos(P)] ])
        C_r = np.array([[1.0, 0.0,         0.0         ],
                        [0.0, math.cos(R), -math.sin(R)],
                        [0.0, math.sin(R),  math.cos(R)] ])
        return C_h @ C_p @ C_r


    def measureMarker(self, img, nav=None):
        """
        Looks for an aruco marker in jpg, draws axes and returns
        a list of dictionaries containing the measurements.
        Also keeps a copy of the image with the axes drawn on so it
        can be shown to the user. Use the self.jpg() function to
        get the latest image.
        If nav is a dictionary containing LLA+HPR then unmapped markers
        will be computed.
        """
        # List of dictionaries with marker measurements
        marker_measurements = []
        
        # This is in a try block because many things can go wrong
        try:        
            # Image size has to match the calibration resolution
            if self.calibrationResolution != None and self.calibrationResolution != img.shape[0:2]:
                raise Exception(f"Calibration resolution {self.calibrationResolution} does not match image resolution {img.shape[0:2]}")
            
            # Detect the markers using the new ArucoDetector API
            markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(img)

            # Test to see if markers are detected
            if markerIds is not None and len(markerCorners) > 0:
                # Draw the detected markers
                cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
                
                # Find the pose for each marker
                rs, ts, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.10, self.mtx, self.dist)
                                
                # Loop through the markers and compute their position, etc.
                for r, t, idm in zip(rs, ts, markerIds):
                    # Put the axes on the marker using the new OpenCV function
                    cv2.drawFrameAxes(img, self.mtx, self.dist, r, t, 0.1)

                    # Look up the marker in the map file
                    if idm[0] in self.markers:
                        m = self.markers[idm[0]]
                        
                        # Work out what the camera thinks the HPR of the vehicle/body frame is
                        # See above from explanation of the rotations, full chain of rotations reminder:
                        # C_nb = C_nm * C_mM * C_MC * C_Cc * C_cb
                        C_nm = self._C_nb(m['AmHeading'],m['AmPitch'],m['AmRoll'])
                        # self.C_mM static and defined above
                        C_MC = cv2.Rodrigues(r)[0].T
                        # self.C_Cc static and defined above
                        # self.C_cb configured by user
                        C_nb = C_nm @ self.C_mM @ C_MC @ self.C_Cc @ self.C_cb
                        HPR = dcm2euler(C_nb)                       
                        am = m.copy()
                        am.update( {
                            # CM used for camera to marker (not centimetres!)
                            'AmXCM_C' : t[0][0], # Camera to Marker, Camera frame, x direction
                            'AmYCM_C' : t[0][1], # Camera to Marker, Camera frame, y direction
                            'AmZCM_C' : t[0][2], # Camera to Marker, Camera frame, z direction
                            'AmHeading_nb' : HPR[0],  # Heading of body frame (degrees)
                            'AmPitch_nb'   : HPR[1],  # Pitch of body frame (degrees)
                            'AmRoll_nb'    : HPR[2],  # Roll of body frame (degrees)
                            'AmcRodrigues0': r[0][0],
                            'AmcRodrigues1': r[0][1],
                            'AmcRodrigues2': r[0][2],
                            } )

                        marker_measurements.append(am)
                        
                    elif nav is not None:
                        try:
                            # Marker not in map file
                            # Dxm_n = C_nb * (Dxc_b + Dcm_b)
                            #       = C_nb * (Dxc_b + C_bc * C_cC * Dcm_C)
                            # C_nb = C_nm * C_mM * C_MC * C_Cc * C_cb ... normal chain of rotations, so
                            # C_nm = C_nb * (C_mM * C_MC * C_Cc * C_cb).T
                            # TODO: image time and nav time are not aligned
                            #       So there is a time mis-match when nav is used
                            C_nb = self._C_nb(nav['Heading'],nav['Pitch'],nav['Roll'])                        
                            Dcm_b = self.C_cb.T @ self.C_Cc.T @ t[0]
                            Dxm_b = self.Dxc_b + Dcm_b
                            Dxm_n = C_nb @ Dxm_b # [North, East, Down]
                            # Compute the position of the marker in AmLocal
                            # Dam_n ... AmBase to marker in navigation frame (NED)
                            Dam_n = np.array((nav['AmLocalN'],nav['AmLocalE'],nav['AmLocalD'])) + Dxm_n
                            # Compute the angles
                            C_MC = cv2.Rodrigues(r)[0].T
                            C_nm = C_nb @ (self.C_mM @ C_MC @ self.C_Cc @ self.C_cb).T
                            HPR = dcm2euler(C_nm)
                            self.unmappedMarkers[int(idm[0])] = {
                                'AmId':int(idm[0]), 'AmSize':0.1, 'AmN':Dam_n[0], 'AmE':Dam_n[1], 'AmD':Dam_n[2],
                                'AmHeading':HPR[0], 'AmPitch':HPR[1], 'AmRoll':HPR[2]}
                        except:
                            pass # If nav not initialised then quietly go on
            
            if self.show_rejected:
                for candidate in rejectedCandidates:
                    corners = candidate.reshape(-1, 2)  # 4x2 array
                    center = np.mean(corners, axis=0).astype(int)

                    # Compute radius as the max distance from center to any corner
                    distances = np.linalg.norm(corners - center, axis=1)
                    radius = int(np.max(distances))

                    # Draw the circle
                    cv2.circle(img, tuple(center), radius, color=(0, 255, 255), thickness=1)  # yellow circle

            # Notify anything that is using this as a camera
            with self.cam_condition:
                self.cam_frames.append(img)
                self.cam_condition.notify_all()
        except Exception as e:
            logging.exception(traceback.format_exc())
        
        return marker_measurements # List of dictionaries containing the measurements


    def img(self):
        """
        Returns an image with the markers and axes drawn on
        """
        with self.cam_condition:
            self.cam_condition.wait()
        return self.cam_frames[-1]


# Moved out of the class as it is a useful function for other modules
# TODO: Put it in its own module or a module with other useful functions
def dcm2euler(dcm):
    """
    Computes the HPR euler angles from a direction cosine
    matrix Rbn
    This function follows the OxTS NCOM decoder C code driver:
    https://github.com/OxfordTechnicalSolutions/NCOMdecoder/tree/master/nav
    """
    TINY = 1e-20
    
    # There is no check to see whether the dcm is valid
    # First compute the pitch angle
    if dcm[2][0]  <= -1.0:
        pitch = math.pi/2.0
    elif dcm[2][0] >= 1.0:
        pitch = -math.pi/2.0
    else:
        pitch = math.asin(-dcm[2][0])
    
    # Compute the cosine of pitch angle from dcm
    cos_ph2 = dcm[0][0] * dcm[0][0] + dcm[1][0] * dcm[1][0]
    cos_pr2 = dcm[2][1] * dcm[2][1] + dcm[2][2] * dcm[2][2]
    if cos_ph2 > TINY or cos_pr2 > TINY:
        # Standard formulation
        heading = math.atan2( dcm[1][0], dcm[0][0] )
        roll = math.atan2( dcm[2][1], dcm[2][2] )
    elif cos_ph2 > cos_pr2:
        # Assume heading angle estimate is better conditioned
        if pitch > 0.0:
            x = math.atan2(dcm[1][0]-dcm[0][1], dcm[0][2]+dcm[1][1])
            if cos_ph2 == 0.0:
                roll = 0.0
                heading = x
            else:
                heading = math.atan2(dcm[1][0], dcm[0][0])
                roll = heading - x
                if roll > math.pi:
                    roll = roll - 2.0 * math.pi
                elif roll < -math.pi:
                    roll = roll + 2.0 * math.pi
        else:
            x = math.atan2(-dcm[1][2]-dcm[0][1], dcm[1][1]-dcm[0][2])
            if cos_ph2 == 0.0:
                roll = 0.0
                heading = x
            else:
                heading = math.atan2(dcm[1][0], dcm[0][0])
                roll = x - heading
                if roll > math.pi:
                    roll = roll - 2.0 * math.pi
                elif roll < -math.pi:
                    roll = roll + 2.0 * math.pi             
    else:
        # Assume roll angle estimate is better conditioned
        if pitch > 0.0:
            x = math.atan2(dcm[1][2]-dcm[0][1], dcm[0][2]+dcm[1][1])
            if cos_pr2 == 0.0:
                heading = 0.0
                roll = -x
            else:
                roll = math.atan2(dcm[2][1],dcm[2][2])
                heading = x + roll
                if heading > math.pi:
                    heading = heading - 2.0 * math.pi
                elif heading < -math.pi:
                    heading = heading + 2.0 * math.pi
        else:
            x = math.atan2(-dcm[1][2]-dcm[0][1], dcm[1][1]-dcm[0][2])
            if cos_pr2 == 0.0:
                roll = x
                heading = 0.0
            else:
                roll = math.atan2(dcm[2][1], dcm[2][2])
                heading = x - roll
                if heading > math.pi:
                    heading = heading - 2.0 * math.pi
                elif heading < -math.pi:
                    heading = heading + 2.0 * math.pi
    # 250731 BW: After all that conditioning of heading, change it to 0 to 2*pi
    if heading < 0.0: heading += 2.0 * math.pi
    return np.array((heading,pitch,roll))*180.0/math.pi

