# gad_aruco.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Module to take the aruco markers and send them as GAD updates to the OXTS navigation system.
"""
import threading
import oxts_sdk
import time
import numpy as np
import ncomrx
import aruco
import logging
import datetime
from config import CFG

# Definitions for where the camera is and how it is orientated compared
# to the xNAV.

# For a rotated camera (i.e. one where the camera z-axis isn't aligned
# to the xNAV x-axis):
# C --> Camera axes (X/row 1 points right in the image frame. Y/row 2 points down in the image frame)
# c --> OxTS-style, but still rotated by H_c, P_c, R_c (camera HPR)
# b --> OxTS body frame
# V_b = C_nb(H_c, P_c, R_c) * C_cC * V_C

# Just for explicit clarity (if English allows):
# C_Cc: Rotation matrix from OxTS-style camera frame ("c") to OpenCV-style camera frame ("C")
# "c" frame axes (OxTS convention):
#   - X_c: forward (vehicle direction)
#   - Y_c: right (right side)
#   - Z_c: down (toward road)
# "C" frame axes (OpenCV convention):
#   - X_C: right in image
#   - Y_C: down in image
#   - Z_C: forward (into scene)
# "b" frame axes (OxTS body frame, or vehicle frame)
# ... is different to c because the camera might be orientated differently in the vehicle.
# We could do a transformation straight from "C" to "b", but I find it easier to
# put H_c, P_c and R_c in the configuration file, and have them with (normally) trivial rotations
# The GAD update is in the OXTS IMU frame, which is not the same as the body frame, and is
# defined by mobile.vat n the OXTS configuration files.

# To work out C_bc and C_bi:
# Start thinking from the "c" or the "i" directions
#   Rotate through Z - this is the heading, right thumb along Z axis, fingers in positive rotation direction
#   Rotate through Y - this is the pitch
#   Rotate through X - this is the roll
# so that you end up in the "b" frame
#
# The full chain of rotations is such that:
#   C_nb = C_nm * C_mM * C_MC * C_Cc * C_cb
# where:
#   C_nm - navigation to marker frame, in the map file as HmPmRm
#   C_mM - definitions, see axes above
#   C_MC - (C_CM = Rodrigues()).T from aruco code
#   C_Cc - definitions, see axes above
#   C_cb - camera mount in the vehicle

class GadAruco:
    """
    GadAruco class to send aruco marker updates as GAD updates to the OXTS navigation system.
    This is used to keep the xNAV650 stable if GNSS is not available, etc.
    """
    def __init__(self, CFG):
        self.CFG = CFG
        self.nrxs = None
        self.ws = None
        self.cam = None

        # Various transfoms
        self.C_Cc = np.array([[0.0,1.0,0.0], [0.0,0.0,1.0], [1.0,0.0,0.0]]) # oxts-camera to open-cv-camera
        self.C_cb = ncomrx.RbnHPR(CFG['HPR_cb'])   # From oxts-body to oxts-camera frame
        self.C_ib = ncomrx.RbnHPR(CFG['HPR_ib'])   # From oxts-body to oxts-imu (vehicle attitude file)
        self.Dxc_b = np.array(CFG['Dxc_b'])        # xNav to camera displacement, body frame

        self.ar = aruco.Aruco(C_cb=self.C_cb, Dxc_b=self.Dxc_b)     # Aruco decoder

        self.updateAmGo = True # Can be used to end the aruco marker thread
        self.mapUpdatePeriod = 10 # Seconds between map being sent
        self.gad_heading = 'heading'
        self._am_thread = threading.Thread(target=self.updateAm, daemon=True)
        self._am_thread.start() # Starts decoding and sending images


    def updateAm(self):
        """
        Calls measureMarker, formats the data and sends it to the web page.
        Performs generic aiding update for the marker
        """
        # Handler for gad updates to xNAV
        gh = oxts_sdk.GadHandler()
        gh.set_encoder_to_bin()
        gh.set_output_mode_to_udp(self.CFG['InsIp']) 
        
        # Map won't be sent every cycle as it could be quite big
        timeToSendMap = time.time()
        
        while self.updateAmGo:
            try:
                if self.cam is None or self.nrxs is None:
                    # Wait for the camera and ncom receiver to be set up
                    time.sleep(0.5)
                    continue

                # Get the jpg from the camera and find the marker position
                # mt is machine time of the image
                # ex is the exposure time of the image
                jpg, mt, ex = self.cam.img2()

                # Process the image
                # ams is a list of dictionaries containing all aruco markers
                # measurements (NED, HPR, etc.)
                ams = self.ar.measureMarker(jpg, nav=self.nrxs.nrx[self.CFG['InsIp']]['decoder'].nav)
                for am in ams:
                    try:
                        # Timing - add these to the marker dictionary
                        # for display on webpage
                        am['ImageTime'] = mt
                        am['ExposureTime'] = ex
                        am['MachineTime'] = time.monotonic()
                        nrx = self.nrxs.nrx[self.CFG['InsIp']]['decoder']
                        imGpsWeek, imGpsSeconds  = nrx.mt2Gps(mt)
                        am['ImageGpsTime'] =  ncomrx.GPS_STARTTIME + \
                            datetime.timedelta( minutes=imGpsWeek*10080, seconds=imGpsSeconds )

                        # Note: {XYZ}CM measurements are in camera co-ordinates XC, YC, ZC
                        # CM used for camera to marker (not centimetres!)
                        # Camera axes X: right, Y: down, Z: forward                        
                        # Compute marker position in body/vehicle frame of xNAV
                        DCM_C = np.array([am['AmXCM_C'],am['AmYCM_C'],am['AmZCM_C']]) # Camera to marker, camera frame
                        DxM_b = self.Dxc_b + self.C_cb.T @ self.C_Cc.T @ DCM_C        # xNAV to marker in body/vehicle frame
                        DxM_i = self.C_ib @ DxM_b

                        am['AmXCM_b'] = DxM_b[0]
                        am['AmYCM_b'] = DxM_b[1]
                        am['AmZCM_b'] = DxM_b[2]
                        
                        # Transform AmN, AmE, AmD to geodetic frame using AmBaseLLA
                        LLA = ncomrx.NED2LLA(am['AmN'], am['AmE'], am['AmD'], CFG['AmBaseLLA'][0], CFG['AmBaseLLA'][1], CFG['AmBaseLLA'][2])

                        # Perform geodetic generic aiding update
                        gp = oxts_sdk.GadPosition(129)
                        gp.pos_geodetic = [ LLA['Lat'], LLA['Lon'], LLA['Alt']]
                        # The variance is from the xNAV to the camera
                        # and the marker accuracy
                        # 0.0001 ~ 1cm, though I have probably measured it more accurately than this
                        gp.pos_geodetic_var = [0.001,0.001,0.001,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                        # TODO: Needs further work to get timing accurate to ~ 1ms
                        # Currently timing of NCOM is from Ethernet, which has a latency
                        # Timing of the camera is from the GPU when the frame starts arriving
                        # but the image is from an earlier time than this
                        # Needs calibration
                        gp.time_gps = [imGpsWeek, imGpsSeconds]
                        gp.aiding_lever_arm_fixed = DxM_i
                        # Empirical formula for lever arm measurement
                        # At 1m, 3cm is OK
                        # At 2m, 10cm; 3m 30cm; 4m 1m
                        # Use 0.0001 * 10^range
                        d = np.linalg.norm(DxM_i,2)
                        d = np.clip(d,1.0,3.0)
                        a = 0.00001 * 30**d
                        gp.aiding_lever_arm_var = [a,a,a]
                        gh.send_packet(gp)
                        
                        if self.gad_heading == 'heading':
                            # This works but, without an innovation output, it is hard to see it working
                            # Also, with heading lock active, it is virtually impossible to notice it working
                            ga = oxts_sdk.GadHeading(131)
                            ga.heading = am['AmHeading_nb']          
                            ga.heading_var = 100.0 # About 10 degrees, which is fine because update is correlated and frequent
                            # See comment on timing of position
                            ga.time_gps = [imGpsWeek, imGpsSeconds]
                            ga.aiding_alignment_fixed = CFG["HPR_ib"]
                            ga.aiding_alignment_var = [5.0,5.0,5.0] # Not very aligned
                            gh.send_packet(ga)
                        elif self.gad_heading == 'orientation':
                            # I have been unable to get this to work correctly
                            ga = oxts_sdk.GadAttitude(131)
                            ga.att = [ am['AmHeading_nb'], am['AmPitch_nb'], am['AmRoll_nb'] ]                   
                            ga.att_var = [100.0,1000.0,1000.0]
                            # See comment on timing of position
                            ga.time_gps = [imGpsWeek, imGpsSeconds]
                            ga.aiding_alignment_fixed = CFG["HPR_ib"]
                            ga.aiding_alignment_var = [5.0,5.0,5.0]
                            gh.send_packet(ga)

                        elif self.gad_heading == 'orientation2':
                            # I have been unable to get this to work correctly
                            ga = oxts_sdk.GadAttitude(131)
                            C_nb = ncomrx.RbnHPR((am['AmHeading_nb'], am['AmPitch_nb'], am['AmRoll_nb']))
                            C_ni = C_nb @ self.C_ib.T
                            HPR_i = aruco.dcm2euler(C_ni)
                            ga.att = [ HPR_i[0], HPR_i[1], HPR_i[2] ]
                            Var_i = self.C_ib @ np.diag([100.0,1000.0,1000.0]) @ self.C_ib.T           
                            print(HPR_i, [Var_i[0][0], Var_i[1][1], Var_i[2][2]])
                            ga.att_var = [Var_i[0][0], Var_i[1][1], Var_i[2][2]]
                            # See comment on timing of position
                            ga.time_gps = [imGpsWeek, imGpsSeconds]
                            ga.aiding_alignment_fixed = [0.0, 0.0, 0.0]
                            ga.aiding_alignment_var = [5.0,5.0,5.0]
                            gh.send_packet(ga)

                        # Send marker information to the webpage
                        if self.ws is not None:
                            self.ws.send("am-data", am) # Send to web server
                                                        
                    except Exception as e:
                        logging.exception(datetime.datetime.now())
                
                # Map for web page, sent occasionally
                t = time.time()
                if t > timeToSendMap:
                    am = {}
                    am['AmBaseLLA'] = self.CFG['AmBaseLLA']
                    am['AmMap'] = self.ar.markers
                    am['AmUnmappedMarkers'] = self.ar.unmappedMarkers

                    # Send marker information to the webpage
                    if self.ws is not None:
                        self.ws.send("am-data", am) # Send to web server            
                    
                    timeToSendMap = t + self.mapUpdatePeriod
            except:
                logging.exception("[gad_aruco]: Exception in loop")
                time.sleep(5.0) # Pause loop: a common problem is that the INS is not yet active

    def user_command(self, message):
        if message == "{gad heading":
            self.gad_heading = 'heading'
        elif message == "{gad orientation":
            self.gad_heading = 'orientation'
        elif message == "{gad orientation2":
            self.gad_heading = 'orientation2'
        elif message == "{show_rejected":
            self.ar.show_rejected = True
        elif message == "{hide_rejected":
            self.ar.show_rejected = False


# Additional calculations for ncomrx
# Called (from ncomrx.py) each time an NCOM packet is received
# Add in AmLocal
def calcAmLocal( nrx ):
    global CFG
    AmLocal = ncomrx.LLA2NED(nrx.nav['Lat'] * ncomrx.RAD2DEG,
        nrx.nav['Lon'] * ncomrx.RAD2DEG, nrx.nav['Alt'],
        CFG['AmBaseLLA'][0], CFG['AmBaseLLA'][1], CFG['AmBaseLLA'][2])

    nrx.nav['AmLocalN'] = AmLocal['LocalN']
    nrx.nav['AmLocalE'] = AmLocal['LocalE']
    nrx.nav['AmLocalD'] = AmLocal['LocalD']
    nrx.nav['AmLocalZ'] = AmLocal['LocalZ']   
