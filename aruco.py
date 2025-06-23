# The MIT License (MIT)

# Copyright (C) 2021 s7711
# 39369253+s7711@users.noreply.github.com

# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# This software includes modules with different licenses. See the
# license for each module. For example opencv is licensed under Apache 2
# and the aruco marker functions have a non-commercial license


"""
aruco.py
Looks for aruco markers, computes the distance and angles.

Reads marker map.csv file, csv format. The map file can either have
latitude, longtitude and altitude (LLA) or XYZ fields. The map should
have the following columns:
AmId        - Matching the Aruco dictionary
AmSize      - Metres, currently ignored
--- Either LLA using these columns...
AmLat       - Decimal degrees 
AmLon       - Decimal degrees
AmAlt       - Metres above datum
--- Or XYZ using these columns...
AmX         - Metres from origin
AmY         - Metres from origin
AmZ         - Metres from origin
---
AmHeading   - Decimal degrees
AmPitch     - Decimal degrees
AmRoll      - Decimal degrees

Notes on the rotations. Help from web pages:
  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
The camera co-ordinates are on page:
  https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html

OxTS HPR (Heading, Pitch, Roll) relate vectors in the body frame
to the navigation frame:
  V_n = Rbn(HPR) * V_b
where:
  V_n is a vector with world-frame axes (north, east down)
  V_b is the same vector with body frame axes (XYZ)
  Rbn(HPR) is defined in the RT user manual

For OxTS V_b has:
  V_b[0] forward (usually denoted as x)
  V_b[1] right (usually denoted as y)
  V_b[2] down (usually denoted as z)
  V_n[0] north
  V_n[1] east
  W_n[2] down
  R_bn(HPR) = R_h * R_p * R_r

The camera co-ordinates are very different, with:
  X_c right (viewed from behind the camera)
  Y_c down
  Z_c along the camera axis towards the scene

The marker co-ordinates are:
  X_m right (from the front of the marker)
  Y_m up
  Z_m towards the camera (looking from behind the marker)

The definition of the marker HmPmRm (marker heading, pitch, roll
in the map file) are in the OxTS convention so we need
to relate X_m, Y_m, Z_m to the "marker body"
X_M, Y_M, Z_M. For a marker with Hm=0, Pm=0 and Rm=0 then:
  X_m points east (which is X_M)
  Y_m points up (which is -Z_M)
  Z_m points south (which is -Y_M)

Aruco estimates the pose:
"marker coordinate system to the camera coordinate system"
written as R_mc (or rotation from marker to camera)

The relation between vectors is therefore:
  V_n = R_bn(HmPmRm) * R_Mm * R_mc.T * R_bc * V_b
(R_mc.T is the transpose, giving R_cm) And so:
  R_hpr = R_bn(HmPmRm) * R_Mm * R_mc.T * R_bc
And the Euler angles can be found from R_hpr
"""

import threading
import collections
import math
import cv2
import numpy as np
import csv
import os
import traceback

import ncomrx # for useful functions

class Aruco(object):
    """
    Looks for Aruco markers, computes the vector and angles ready to
    send to an OxTS INS
    """
    def __init__(self, mapfile='map.csv', R_Cb=None, Db_xc=None, AmBaseLLA=None):
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
            print("Camera calibration resolution unknown")
            self.calibrationResolution = None
        self.mtx = fs.getNode("camera_matrix").mat()
        self.dist = fs.getNode("distortion_coefficients").mat()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        try:
            self.parameters = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Use the new ArucoDetector class
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Method of stopping the processing and terminating the thread
        self.go = True
        
        # Load the mapfile
        # markers is a dictionary of the markers where the key is the
        # marker id and the value is a dictionary of the marker
        # information from the CSV file, see comment at the top of the
        # file
        # For example { 201 : { AmId:201, AmSize:0.1, AmLat: 51.2, AmLon:-1.0, ... },
        #                19 : { AmId:19,  AmSize:0.2, AmLat: 51.3, AmLon:-1.1, ... } }
        self.markers = {}
        try:
            with open(mapfile, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    self._numberify(row)
                    # For map files with Lat, Lon, Alt, compute XYZ (i.e. NED) in AmBase frame
                    if AmBaseLLA is not None and "AmLat" in row:                    
                        markerNED = ncomrx.LLA2NED(row['AmLat'], row['AmLon'], row['AmAlt'],
                            AmBaseLLA[0], AmBaseLLA[1], AmBaseLLA[2])
                        row['AmX'] = markerNED['LocalE']
                        row['AmY'] = markerNED['LocalN']
                        row['AmZ'] = markerNED['LocalZ']
                    self.markers.update({row['AmId']:row})
        except Exception as e:
            print(traceback.format_exc())

        # Rotation matricies so the camera rotation can be expressed as HPR
        # R_Mm - vector in HmPmRm from a vector in marker co-ordinates
        self.R_Mm = np.array([[ 0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [ 0.0,-1.0, 0.0]])
        
        # R_bc - Rotation of vector in OxTS body frame to camera frame
        # This is for a normal orientation camera (not upside down)
        # with photo along x-axis of the OxTS INS (image forward)
        R_Cc = np.array([[0.0,1.0,0.0],
                         [0.0,0.0,1.0],
                         [1.0,0.0,0.0]])
        self.R_bc = R_Cc if R_Cb is None else (R_Cc.T.dot(R_Cb)).T
        self.Db_xc = np.zeros(3) if Db_xc is None else Db_xc
        
        # Unmapped markers seen by the camera
        # Put in the map format
        # For example { 201 : { AmId:201, AmSize:0.1, AmLat: 51.2, AmLon:-1.0, ... },
        #                19 : { AmId:19,  AmSize:0.2, AmLat: 51.3, AmLon:-1.1, ... } }
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




    def _Rbn(self,H,P,R):
        """
        Computes the direction cosine matrix rotation from body to navigation
        frame from Heading, Pitch and Roll (in degrees), such that:
        Vn = Rbn(HPR) * Vb
        Where Vn is the vector in the navigation frame (North, East, Down)
        and Vb is the vector in the body frame (X-forward, Y-right, Z-down)
        See RT User Manual page 101 for details:
        https://www.oxts.com/wp-content/uploads/2020/03/rtman-200302.pdf
        (page 101 in version 200220)
        """
        H = H * math.pi/180.0
        P = P * math.pi/180.0
        R = R * math.pi/180.0
        # Rotation from body to navigation frame using Heading, Pitch, Roll in radians
        Rh = np.array([[math.cos(H), -math.sin(H), 0.0],
                       [math.sin(H),  math.cos(H), 0.0],
                       [0.0,          0.0,         1.0] ])
        Rp = np.array([[ math.cos(P), 0.0, math.sin(P)],
                       [ 0.0,         1.0,         0.0],
                       [-math.sin(P), 0.0, math.cos(P)] ])
        Rr = np.array([[1.0, 0.0,         0.0         ],
                       [0.0, math.cos(R), -math.sin(R)],
                       [0.0, math.sin(R),  math.cos(R)] ])
        return Rh.dot(Rp).dot(Rr)


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
                rs, ts, objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.10, self.mtx, self.dist)
                                
                # Loop through the markers and compute their position, etc.
                for r, t, idm in zip(rs, ts, markerIds):
                    # Put the axes on the marker using the new OpenCV function
                    cv2.drawFrameAxes(img, self.mtx, self.dist, r, t, 0.1)

                    # Look up the marker in the map file
                    if idm[0] in self.markers:
                        m = self.markers[idm[0]]
                                   
                        # See above from explanation of the rotations
                        # R_hpr = R_bn(HmPmRm) * R_Mm * R_mc.T * R_bc
                        R_mc, J = cv2.Rodrigues(r)
                        R_nM = self._Rbn(m['AmHeading'],m['AmPitch'],m['AmRoll'])                        
                        hpr = dcm2euler(R_nM.dot(self.R_Mm).dot(R_mc.T).dot(self.R_bc))
                        am = m.copy()
                        am.update( {
                            # cm used for camera to marker (not centimetres!)
                            'AmXc_cm' : t[0][0], # Camera to Marker, Camera frame, x direction
                            'AmYc_cm' : t[0][1], # Camera to Marker, Camera frame, y direction
                            'AmZc_cm' : t[0][2], # Camera to Marker, Camera frame, z direction
                            'AmcHeading' : hpr[0],  # Heading of body frame (degrees)
                            'AmcPitch'   : hpr[1],  # Pitch of body frame (degrees)
                            'AmcRoll'    : hpr[2],  # Roll of body frame (degrees)
                            'AmcRodrigues0': r[0][0],
                            'AmcRodrigues1': r[0][1],
                            'AmcRodrigues2': r[0][2],
                            } )

                        marker_measurements.append(am)
                        
                    elif nav is not None:
                        try:
                            # Marker not in map file
                            # Dn_xm = R_bn * (Db_xc + Db_cm)
                            #       = R_bn * (Db_xc + R_cb * Dc_cm)
                            # TODO: image time and nav time are not aligned
                            #       So there is a time mis-match when nav is used
                            R_bn = ncomrx.RbnHPR(nav['Heading'],nav['Pitch'],nav['Roll'])                        
                            Db_cm = self.R_bc.T.dot(t[0])
                            Db_xm = self.Db_xc + Db_cm
                            Dn_xm = R_bn.dot(Db_xm) # [North, East, Down]
                            # Compute the position of the marker in AmLocal
                            # Dn_am ... AmBase to marker in navigation frame (NED)
                            Dn_am = np.array((nav['AmLocalN'],nav['AmLocalE'],nav['AmLocalD'])) + Dn_xm
                            # TODO: Naming convention confusion between NAD and XYZ needs fixing (in other places too)
                            self.unmappedMarkers[int(idm[0])] = {
                                'AmId':int(idm[0]), 'AmSize':0.1, 'AmX':Dn_am[1], 'AmY':Dn_am[0], 'AmZ':Dn_am[2] }
                        except:
                            pass # If nav not initialised then quietly go on
            
            # Notify anything that is using this as a camera
            with self.cam_condition:
                self.cam_frames.append(img)
                self.cam_condition.notify_all()
        except Exception as e:
            print(traceback.format_exc())
        
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
    return np.array((heading,pitch,roll))*180.0/math.pi

