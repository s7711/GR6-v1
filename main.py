# main.py
# Licensed under the MIT License – see LICENSE file for details.

"""
GR6-v1 entry point

Usgae:
    python3 main.py

Then from a web browser:
    http://<ip address>:8000

This program is currently under development. It has been inherited from
a previous project and several parts probably do not work as expected.
"""

# Standard python imports
import time
import datetime
import threading
import socket
import sys
import os
import re
import queue
import math
import numpy as np
import logging
import subprocess

# OxTS generic aiding module.
# See https://github.com/OxfordTechnicalSolutions/gad-sdk
import oxts_sdk

# Local modules
import ncomrx_thread
import web_server
import bgCamera2
import aruco
import ncomrx
import config
import gad_fake

logging.info("Program starting") # Makes sure that the default logging handler is set up
CFG = config.ReadConfig() # Read the configuration file


# Definitions for where the camera is and how it is orientated compare
# to the xNAV.

# For a rotated camera (i.e. one where the camera z-axis isn't aligned
# to the xNAV x-axis):
# c --> Camera axes
# C --> OxTS-style, but still rotated by H_C, P_C, R_C (camera HPR)
# b --> OxTS body frame
# V_b = Rbn(H_C, P_C, R_C) * R_Cc * V_c

R_Cc = np.array([[0.0,1.0,0.0], [0.0,0.0,1.0], [1.0,0.0,0.0]]) # oxts-camera to open-cv-camera
R_Cb = ncomrx.RbnHPR(CFG['HPR_Cb']) # From oxts-camera to oxts-body frame
# R_cb = R_cC * R_Cb         # R_cC = R_Cc.T
R_cb = R_Cc.T.dot(R_Cb)      # opencv-camera to oxts-body frame
Db_xc = np.array(CFG['Db_xc']) # xNav to camera displacement, body frame


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
         

# Start up the services
nrxs = ncomrx_thread.NcomRxThread()    # Background ncom receiver and decoder
nrxs.moreCalcs.append(calcAmLocal)     # Add in calculation for AmLocal co-ordinates
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Socket for sending UDP

cam = bgCamera2.BgCamera() # Opens the camera in the background
ws = web_server.WebServer() # Starts a web server in the background using Flask-SocketIO
ar = aruco.Aruco(R_Cb=R_Cb, Db_xc=Db_xc, AmBaseLLA=CFG['AmBaseLLA'])     # Aruco decoder
ws.set_camera(ar.img)          # web server /camera.mjpg will use this image

def serve_json():
    """
    serve_json() loops forever serving ncom to the web socket.
    Run in a new thread
    """
    global nrx, ws
    while True:
        time.sleep(0.5) # Sets the update rate for the messages
        
        # nrxs.nrx is a dictionary of all the INSs found on the network
        # ... and the keys are the IP addresses
        # Form a list of the IP addresses
        devices = [ nrx for nrx in nrxs.nrx ] # list of keys/ip addresses
        ws.send("devices", devices) # Send to web server
        
        # For each INS extract the information from the decoder (NcomRx type)
        # Note that this implementation encodes everything in JSON, even if
        # it ends up that no websocket wants the data
        for addr, nrx in nrxs.nrx.items():      
            ws.send("nav-data-"+addr, nrx['decoder'].nav) # Send to web server
            ws.send("nav-status-"+addr, nrx['decoder'].status)
            ws.send("nav-connection-"+addr, nrx['decoder'].connection)

# TODO: above: sort out how the IP address is going to be filtered    


updateAmGo = True # Can be used to end the aruco marker thread
mapUpdatePeriod = 10 # Seconds between map being sent
timeUpdatePeriod = 3 # When INS doesn't have valid time, wait between sending our time

def updateAm():
    """
    Calls measureMarker, formats the data and sends it to the web page.
    Performs generic aiding update for the marker
    """
    
    # Handler for gad updates to xNAV
    gh = oxts_sdk.GadHandler()
    gh.set_encoder_to_bin()
    gh.set_output_mode_to_udp(CFG['InsIp']) 
    
    # Map won't be sent every cycle as it could be quite big
    timeToSendMap = time.time()
    timeToSendInit = time.time() + 3 # Initialise to something
    
    while updateAmGo:
        # Get the jpg from the camera and find the marker position
        # mt is machine time of the image
        # ex is the exposure time of the image
        jpg, mt, ex = cam.img2()

        # Process the image
        # ams is a list of dictionaries containing all aruco markers
        # measurements (XYZ, HPR, Lat, Lon, Alt, etc.)
        ams = ar.measureMarker(jpg, nav=nrxs.nrx[CFG['InsIp']]['decoder'].nav)
        for am in ams:
            try:
                # Timing - add these to the marker dictionary
                # for display on webpage
                am['ImageTime'] = mt
                am['ExposureTime'] = ex
                am['MachineTime'] = time.perf_counter()
                nrx = nrxs.nrx[CFG['InsIp']]['decoder']
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
                Db_xm = Db_xc + R_cb.dot(Dc_cm)                              # xNAV to marker in xNAV frame
                
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
                R_Mb = R_Cb.dot(R_MC)
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
                ws.send("am-data", am) # Send to web server
            
                # Test for initialisation
                t = time.time()
                if( nrx.status['NavStatus'] == 1
                and t > timeToSendInit ):
                    # Need to send time
                    t1 = (datetime.datetime.now(datetime.timezone.utc) - ncomrx.GPS_STARTTIME).total_seconds()
                    gpsWeek = math.floor(t1 / 604800)
                    gpsSeconds = t1 % 604800
                    message = f"!set time target {gpsWeek} {gpsSeconds:.0f}"
                    logging.info(message)
                    sock.sendto(bytes(message+"\n", "utf-8"), (CFG['InsIp'],3001))
                    timeToSendInit = t + timeUpdatePeriod
                if( nrx.status['NavStatus'] == 2
                and t > timeToSendInit ):
                    # Need to initialise
                    # TODO: Offset from the marker needs to be included
                    # Shouldn't use the marker's position!
                    # TODO: Use AmX, Amy, AmZ XY map being used
                    message = f"!set init aidpos {am['AmLat']} {am['AmLon']} {am['AmAlt']}"
                    logging.info(message)
                    sock.sendto(bytes(message+"\n", "utf-8"), (CFG['InsIp'],3001))
                    
                    # TODO: This assumes INS is stationary. Make work
                    # for moving INSs
                    message = f"!set init aidvel 0 0 0"
                    logging.info(message)
                    sock.sendto(bytes(message+"\n", "utf-8"), (CFG['InsIp'],3001))
                    
                    message = f"!set init hea {am['AmbHeading']:.1f}"
                    logging.info(message)
                    sock.sendto(bytes(message+"\n", "utf-8"), (CFG['InsIp'],3001))
                    
                    timeToSendInit = t + timeUpdatePeriod
                    
                        
            except Exception as e:
                logging.exception(datetime.datetime.now())
        
        # Map for web page, sent occasionally
        t = time.time()
        if t > timeToSendMap:
            am = {}
            am['AmBaseLLA'] = CFG['AmBaseLLA']
            am['AmMap'] = ar.markers
            am['AmUnmappedMarkers'] = ar.unmappedMarkers

            # Send marker information to the webpage
            ws.send("am-data", am) # Send to web server            
            
            timeToSendMap = t + mapUpdatePeriod




def upload_xnav_config():
    """ Upload the configuration from the xNAV and store it in
    folder static/xnav-config
    ".txt" is added as an extension so it can be viewed from
    the web browser
    """
    #TODO: This only works in Linux, not Windows
    if sys.platform == 'win32':
        return
    
    time.sleep(20.0) # Don't do this immediately
    
    fs = ["mobile.cfg", # main configuration file
          "mobile.gap", # GNSS antenna position
          "mobile.gpa", # GNSS antenna position accuracy
          "mobile.vat", # Vehicle attitude
          "mobile.vaa", # Vehicle attitude accuracy
          "mobile.att", # (GNSS) antenna attitude
          "mobile.ata", # (GNSS) antenna attitude accuracy
          # TODO: There are several other configuration files
          # that need to be added
          ]
    
    # Create/remove files in the directory
    cp = subprocess.run( ['mkdir', 'static/xnav-config'] )
    cp = subprocess.run( 'rm static/xnav-config/*', shell=True )
    
    for f in fs:
        cp = subprocess.run( ['curl', f'ftp://{CFG["InsIp"]}/{f}', '-o',
          f'static/xnav-config/{f}.txt'], capture_output=True )
        if cp.returncode == 0:
            logging.info(f"Uploaded config file {f}")
        else:
            logging.infor(f"Cannot upload config file {f}")


#####################################################################
# Start the program
logging.info("Use Ctrl-C to quit")
threading.Thread(target=serve_json).start()
gf = gad_fake.GadFake(CFG) # Starts the GAD fake thread
threading.Thread(target=updateAm).start() # Starts decoding and sending images

threading.Thread(target=upload_xnav_config).start()


try:
    # receive messages from web sockets
    while True:
        message,path = ws.recv() # Note: blocking
        logging.info(path + ": " + message)
        if message == '#shutdown':  # Shutdown RaspPi
            subprocess.run('sudo shutdown now', shell=True)
            continue
        elif message == '#internet-passthrough':
            subprocess.call('./shareWlan1Eth0')
            continue          

        # Many ways to split out the query, none particularly elegant
        ip1 = re.search(r'[?&]ip(=([^&#]*)|&|#|$)',path)
        if not ip1: continue        # query not found
        ip2 = ip1.group()           # ?ip=...
        if len(ip2) < 4: continue   # probably not necessary
        ip = ip2[4:]                # ...
        if message == '#send-time':
            t1 = (datetime.datetime.now(datetime.timezone.utc) - ncomrx.GPS_STARTTIME).total_seconds()
            gpsWeek = math.floor(t1 / 604800)
            gpsSeconds = t1 % 604800
            message = f"!set time target {gpsWeek} {gpsSeconds:.0f}"
            logging.info(message)
            sock.sendto(bytes(message+"\n", "utf-8"), (CFG['InsIp'],3001))
        elif message.startswith('#'): # then assume it is a GAD message
            #gad_queue.put((ip,message)) # put in the GAD queue
            gf.userCommand(ip,message) # Use the GAD fake thread
        else:   
            try:                      # because might not be valid IP
                sock.sendto(bytes(message+"\n", "utf-8"), (ip,3001))
                logging.info(f"Sent message {message} to {ip}")
            except:
                logging.exception(datetime.datetime.now())

except KeyboardInterrupt as e:
    print('Stopping')
    # Needs extra code to stop threads, which may be blocked on sockets
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(1)
