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
import math
import logging
import datetime
import socket

# Definitions for where the camera is and how it is orientated compared
# to the xNAV.

# For a rotated camera (i.e. one where the camera z-axis isn't aligned
# to the xNAV x-axis):
# c --> Camera axes
# C --> OxTS-style, but still rotated by H_C, P_C, R_C (camera HPR)
# b --> OxTS body frame
# V_b = Rbn(H_C, P_C, R_C) * R_Cc * V_c
# R_cb = R_cC * R_Cb         # R_cC = R_Cc.T

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
        self.R_Cc = np.array([[0.0,1.0,0.0], [0.0,0.0,1.0], [1.0,0.0,0.0]]) # oxts-camera to open-cv-camera
        self.R_Cb = ncomrx.RbnHPR(CFG['HPR_Cb']) # From oxts-camera to oxts-body frame
        self.R_cb = self.R_Cc.T.dot(self.R_Cb)      # opencv-camera to oxts-body frame
        self.Db_xc = np.array(CFG['Db_xc']) # xNav to camera displacement, body frame

        self.ar = aruco.Aruco(R_Cb=self.R_Cb, Db_xc=self.Db_xc, AmBaseLLA=CFG['AmBaseLLA'])     # Aruco decoder

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Socket for sending UDP

        self.updateAmGo = True # Can be used to end the aruco marker thread
        self.mapUpdatePeriod = 10 # Seconds between map being sent
        self.timeUpdatePeriod = 3
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
        timeToSendInit = time.time() + 3 # Initialise to something
        
        while self.updateAmGo:
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
            # measurements (XYZ, HPR, Lat, Lon, Alt, etc.)
            ams = self.ar.measureMarker(jpg, nav=self.nrxs.nrx[self.CFG['InsIp']]['decoder'].nav)
            for am in ams:
                try:
                    # Timing - add these to the marker dictionary
                    # for display on webpage
                    am['ImageTime'] = mt
                    am['ExposureTime'] = ex
                    am['MachineTime'] = time.perf_counter()
                    nrx = self.nrxs.nrx[self.CFG['InsIp']]['decoder']
                    imTime = nrx.mt2Gps(mt)
                    am['ImageGpsTime'] =  imTime
                    
                    t1 = (imTime - ncomrx.GPS_STARTTIME).total_seconds()
                    imGpsWeek = math.floor(t1 / 604800)
                    imGpsSeconds = t1 % 604800

                    # Note: measurements are in camera co-ordinates Xc, Yc, Zc
                    # cm used for camera to marker (not centimetres!)
                    # Camera axes X: right, Y: down, Z: forward                        
                    # Compute marker position in body frame of xNAV
                    Dc_cm = np.array([am['AmXc_cm'],am['AmYc_cm'],am['AmZc_cm']]) # Camera to marker, camera frame
                    Db_xm = self.Db_xc + self.R_cb.dot(Dc_cm)                              # xNAV to marker in xNAV frame
                    
                    if 'AmLat' in am:
                        # Assume that AmLon and AmAlt are also in am
                        # Perform geodetic generic aiding update
                        gp = oxts_sdk.GadPosition(129)
                        gp.pos_geodetic = [ am['AmLat'], am['AmLon'], am['AmAlt'] ]
                        # This variance is from the xNAV to the camera
                        # and the marker accuracy
                        # 0.0001 ~ 1cm, though I have probably measured it more accurately than this
                        gp.pos_geodetic_var = [0.001,0.001,0.001,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                        # TODO: Needs further work to get timing accurate to ~ 1ms
                        # Currently timing of NCOM is from Ethernet, which has a latency
                        # Timing of the camera is from the GPU when the frame starts arriving
                        # but the image is from an earlier time than this
                        # Needs calibration
                        gp.time_gps = [imGpsWeek, imGpsSeconds]
                        gp.aiding_lever_arm_fixed = Db_xm
                        # Empirical formula for lever arm measurement
                        # At 1m, 3cm is OK
                        # At 2m, 10cm; 3m 30cm; 4m 1m
                        # Use 0.0001 * 10^range
                        d = np.linalg.norm(Db_xm,2)
                        d = np.clip(d,1.0,3.0)
                        a = 0.00001 * 30**d
                        gp.aiding_lever_arm_var = [a,a,a]
                        gh.send_packet(gp)
                    elif 'AmX' in am:
                        # Assume that AmY and AmZ are also in am
                        # Perform local generic aiding update
                        gp = oxts_sdk.GadPosition(130)
                        gp.pos_local = [ am['AmX'], am['AmY'], am['AmZ'] ]
                        # This variance is from the xNAV to the camera
                        # and the marker accuracy
                        # 0.0001 ~ 1cm, though I have probably measured it more accurately than this
                        gp.pos_local_var = [0.001,0.001,0.001,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                        # TODO: Needs further work to get timing accurate to ~ 1ms
                        # Currently timing of NCOM is from Ethernet, which has a latency
                        # Timing of the camera is from the GPU when the frame starts arriving
                        # but the image is from an earlier time than this
                        # Needs calibration
                        gp.time_gps = [imGpsWeek, imGpsSeconds]
                        gp.aiding_lever_arm_fixed = Db_xm
                        # Empirical formula for lever arm measurement
                        # At 1m, 3cm is OK
                        # At 2m, 10cm; 3m 30cm; 4m 1m
                        # Use 0.0001 * 10^range
                        d = np.linalg.norm(Db_xm,2)
                        d = np.clip(d,1.0,3.0)
                        a = 0.00001 * 30**d
                        gp.aiding_lever_arm_var = [a,a,a]
                        gh.send_packet(gp)
                    
                    # Note: HPR are for R_bn for a camera that is aligned
                    # with Zc = Xb (forward pointing camera)
                    # Need to move to the xNAV-body co-ordinates if camera
                    # is at a different angle
                    # Euler - > dcm -> rotate by R_Cb -> dcm2Euler
                    R_MC = ncomrx.RbnHPR(am['AmcHeading'], am['AmcPitch'], am['AmcRoll'])
                    R_Mb = self.R_Cb.dot(R_MC)
                    HbPbRb = aruco.dcm2euler(R_Mb)
                    am['AmbHeading'] = HbPbRb[0] # Heading of body frame of xNAV
                    am['AmbPitch'] = HbPbRb[1]   # Pitch of body frame of xNAV
                    am['AmbRoll'] = HbPbRb[2]    # Roll of body frame of xNAV
                    
                    ga = oxts_sdk.GadAttitude(131)
                    ga.att = [ am['AmbHeading'], am['AmbPitch'], am['AmbRoll'] ]           
                    # Angles from the camera are a mess
                    # An update once every 10s would be a better option
                    # 10.0 ~ 3 degrees results in an accuracy that is much
                    # better than the camera can realistically give         
                    ga.att_var = [100.0,1000.0,1000.0]
                    # See comment on timing of position
                    ga.time_gps = [imGpsWeek, imGpsSeconds]
                    ga.set_aiding_alignment_optimising()
                    ga.aiding_alignment_var = [5.0,5.0,5.0]
                    gh.send_packet(ga)

                    # Send marker information to the webpage
                    if self.ws is not None:
                        self.ws.send("am-data", am) # Send to web server
                
                    if( nrx.status['NavStatus'] == 2
                    and t > timeToSendInit ):
                        # Need to initialise
                        # TODO: Offset from the marker needs to be included
                        # Shouldn't use the marker's position!
                        # TODO: Use AmX, Amy, AmZ XY map being used
                        message = f"!set init aidpos {am['AmLat']} {am['AmLon']} {am['AmAlt']}"
                        logging.info(message)
                        self.sock.sendto(bytes(message+"\n", "utf-8"), (self.CFG['InsIp'],3001))
                        
                        # TODO: This assumes INS is stationary. Make work
                        # for moving INSs
                        message = f"!set init aidvel 0 0 0"
                        logging.info(message)
                        self.sock.sendto(bytes(message+"\n", "utf-8"), (self.CFG['InsIp'],3001))
                        
                        message = f"!set init hea {am['AmbHeading']:.1f}"
                        logging.info(message)
                        self.sock.sendto(bytes(message+"\n", "utf-8"), (self.CFG['InsIp'],3001))
                        
                        timeToSendInit = t + self.timeUpdatePeriod
                        
                            
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


