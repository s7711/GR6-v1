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

# Version 250623 - Lots of changes and probably some things are broken
#    - Moved to picamera2
#    - Moved to Flash-based web server
#    - New timing based on monotonic, not perf_counter, and not checked
# Version 240209 - Added shutdown to devices webpage; send-time;
#   internet pass-through; AmMap sent on a time (not with images)
# Version 220509 - ncomrx.py decodes local co-ordinates. Map file
#   supports LLA or XYZ and aiding will be either geodetic or local.
#   TODO: XYZ version does not send initialisation commands.
# Version 220506 - HTML changes only. Using chart.js. Websocket moved to
#   messages.js. All styles in styles.css. Pages work without internet.
# Version 220401 - Variable empirical accuracy for pos_geodetic update
#   Using subpixel corners for aruco.
# Version 220302 - Fixed threads breaking websocket
#   Removed static GAD and initialisation from gad.html
#   Send AmBaseLLA from config
#   Initialise from markers
#   Added shutdown command
# Version 220301 - Better exception messages
# Version 220215 - Changed to one web socket. Added Db_xc to config.
#   Added camera rotation (not in ideal way)
# Version 220214 - Added configuration file
# Version 220211 - Changed to 1280x960 resolution. Camera HPR added,
#   pointing left
# Version 220204 - Multiple markers from a mapfile
# Version 220128 - GAD timing from camera time. Lever-arm for
#   camera-marker vector update
# Version 220127 - am.json websocket for camera measurements. Uses
#   camera for GAD update (position, heading)

"""
ncom-web-gad main entry point

The ncom-web-gad application:
* Decodes NCOM ethernet data from OxTS inertial navigation systems
* Serves static web pages
* Has a websocket called message.json that sends information to the
  web pages. The websocket is organised as a dictonary where the key
  gives the type of information. For example the "nav" key sends the
  navigation information from the OxTS inertial navigation system.
* Has a websocket called devices.json, which lists the IP addresses
  of the INSs on the network
* Opens the Raspberry Pi camera, looks for an aruco marker
  then sends a generic aiding update. This version uses camera time.

The configuration is read from three places:
* In this file (main.py), CFG holds a default set of values
* ~/.gad_configuration.json overwrites the default values (if present)
* configuration.json (local folder) overwrites again (if present)

Note that this version is IP address specific and can be used with
multiple INSs on the same network. To select which INS you want the
data from you need to specify a query in the web socket address.
For example:

  ws://192.168.2.123:8000/nav.json?ip=192.168.2.62

will connect to 192.168.2.123 and open the web socket that serves
navigation data from INS on IP address 192.168.2.62

The devices.json web socket doesn't need an IP address because it lists
all of the devices/IP addresses that have been received

HOWEVER the aruco marker update is not automatic and is sent to a fixed
IP address specified in the configuration file.

The basic hardware setup that I used is:

"OxTS <--> Raspberry Pi" connected by ethernet using static IP in range 192.168.2.xxx
"Raspberry Pi <--> network" connected by wlan using DHCP (192.168.1.xxx)

This separates the OxTS navigation system from the main network.

I used a 5MP version 1 camera. Use "camcal" code to calibrate your
camera. Make sure the resolution used for calibration matches the
resolution used here.

Usage:

python3 main.py

Then, from a web browser:

http://<ip of python PC>:8000/index.html
http://<ip of python PC>:8000/nav.html?ip=<ip of INS>
http://<ip of python PC>:8000/status.html?ip=<ip of INS>

For example: http://192.168.1.10:8000/nav.html?ip=195.0.0.20

You can add html pages to directory "static" to create your own
templates. My HTML is functional but it needs someone with more
skill to create beautiful pages :-)

The python http.server that the web server is based on claims not to
be secure so it is probably best not to expose this application to
the web.
"""

# Standard python imports
import time
import datetime
import json
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

logging.info("Program starting") # Makes sure that the default logging handler is set up

# Configuration will be stored in a global dictionary called CFG
# Configurations files will update the default configuration
# Default configuration
CFG = {
    # InsIp
    # This is the IP address where aiding updates will be sent
    "InsIp":"192.168.2.62",
    
    # HPR_Cb, tuple, degrees
    # Heading, Pitch, Roll for body frame from Camera frame
    # For example, if the camera points left then the xNAV body frame
    # has a 90 degree heading rotation from the Camera frame
    # (The camera axes having already been changed to a nominally
    # x-forward, y-right, z-down frame)
    "HPR_Cb":(90,0,0),
    
    # Db_xc, tuple, metres
    # Displacement of xNAV to camera in xNAV body frame
    # For example, if the camera is 10cm forward of the xNAV then
    # (0.1,0.0)
    "Db_xc":(0.0,0.0,0.0),
    
    # BaseLLA, tuble, (deg, deg, m)
    # The latitude, longitude and altitude for X0Y0Z0 needs to be
    # given so the website map can be made
    "AmBaseLLA":(52.0,-1.0,100.0), # Defaults not useful, have in config file
}

# Read configuration files, if one exists
try:
    cfg1 = {}
    homeDir = os.path.expanduser('~')
    with open(os.path.join(homeDir,'.gad_configuration.json')) as f:
        cfg1 = json.load(f)
    for key in cfg1:
        CFG[key] = cfg1[key]
except Exception as e:
    logging.exception(datetime.datetime.now())
try:
    cfg2 = {}
    with open('configuration.json') as f:
        cfg2 = json.load(f)
    for key in cfg2:
        CFG[key] = cfg2[key]
except Exception as e:
    pass # Don't worry if this file is not found

# Write out the configuration that is being used - to the static folder
# so the website can access it
try:
    jStr = json.dumps(CFG, indent=2)
    jFile = open(os.path.join("static","CFG.json"),"w")
    jFile.write(jStr)
    jFile.close()
except Exception as e:
    logging.exception(datetime.datetime.now())



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

# Serve_gad is used for "testing" and is not part of the normal aruco
# marker generic aiding update. It can be used to send a regular fixed
# position, velocity of attitude generic aiding update. This can be
# useful while debugging or during initial testing if the camera or
# markers are not working. The velocity update can be useful (as a
# zero velocity update) to keep the INS roughly stationary while
# markers are not visible.
#
# The loop runs on a timer (see time.sleep(0.5) below). A queue is used
# to send new updates to the thread running serve_gad().

gad_queue = queue.Queue()

def serve_gad():
    """
    Sends oxts gad (generic aiding) messages
    Runs in a new thread
    Receives commands from queue
    """
    gh = oxts_sdk.GadHandler()
    gh.set_encoder_to_bin()    

    # gadPkts is a dictionary of {'type_ip':type.ip address, 'pkt':gad_packet}
    # These are the packets that should be sent out on a regular basis
    # The type_ip field is used so the entry can be found, replaced and remove easily
    # and is formatted as "gp:192.168.2.62", where "gp" is:
    #   gp: gadPosition
    #   gv: gadVelocity
    #   etc.
    # Use split(":") to separate type and ip address
    gadPkts = {}
    
    while True:
        time.sleep(0.5) # Sets the rate for updates

        # Process the gad queue
        # If there is anything in the queue then add it to gadPkts{}
        # (Or remove existing gadPkts entries using "stop")
        more = True
        while more:
            try:
                ip, command = gad_queue.get_nowait()
            except:
                # Assume that the queue is now empty
                more = False
            
            if more == True:
                # Split up the command (uses spaces as a delimiter)
                args = command.split()
                
                # Interpret the command
                # In a try block to catch user errors, and anything else
                try:
                    ### vel_neu
                    if args[0] == '#vel_neu':
                        if len(args) == 5 or len(args) == 4:
                            # #vel_neu vn ve vu [acc]
                            gv = oxts_sdk.GadVelocity(132)
                            gv.vel_neu = [ float(v) for v in args[1:4] ]
                            if len(args) == 5:
                                gv.vel_neu_var = [float(args[4]),float(args[4]),float(args[4]),0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                            else:
                                # Default to a covariance of 0.1 ~ 31.6cm/s
                                gv.vel_neu_var = [0.1,0.1,0.1,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                            gv.set_time_void()
                            gv.aiding_lever_arm_fixed = [0.0,0.0,0.0]
                            gv.aiding_lever_arm_var = [0.01,0.01,0.01]
                            gadPkts["gv:"+ip] = gv
                        elif args[1] == 'stop':
                            del gadPkts["gv:"+ip]
                            
                    ### pos_geo    
                    elif args[0] == '#pos_geo':
                        if len(args) == 4:
                            # #pos_geo lat lon alt
                            gp = oxts_sdk.GadPosition(129)
                            gp.pos_geodetic = [ float(v) for v in args[1:4] ]                    
                            gp.pos_geodetic_var = [0.01,0.01,10.0,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                            gp.set_time_void()
                            gp.aiding_lever_arm_fixed = [0.0,0.0,0.0]
                            gp.aiding_lever_arm_var = [0.001,0.001,0.001]
                            gadPkts["gp:"+ip] = gp
                        elif args[1] == 'stop':
                            del gadPkts["gp:"+ip]
                    
                    ### pos_att
                    elif args[0] == '#att':
                        if len(args) == 4:
                            ga = oxts_sdk.GadAttitude(131)
                            ga.att = [ float(v) for v in args[1:4] ]                    
                            ga.att_var = [10.0,100.0,100.0]
                            ga.set_time_void()
                            ga.set_aiding_alignment_optimising()
                            ga.aiding_alignment_var = [5.0,5.0,5.0]
                            gadPkts["ga:"+ip] = ga
                        elif args[1] == 'stop':
                            del gadPkts["ga:"+ip]
                    
                except Exception as e:
                    # Note this will show an error if "stop" is sent
                    # when there is no generic aiding message matching
                    # the stop request. The exception will be a key
                    # error
                    logging.warning(f"Command error: {e}")            
        
        # Each gad packet that needs to be sent to each ip address
        # in gadPkts with keys 'type_ip' and 'pkt'
        # Set the IP address and then send the packet
        for k,pkt in gadPkts.items():
            try:
                type_ip = k.split(':')
                gh.set_output_mode_to_udp(type_ip[1]) # Should be IP address
                gh.send_packet(pkt)
            except Exception as e:
                logging.warning(e)
                del gadPkts[k]                        # Don't bother with this again
            


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
threading.Thread(target=serve_gad).start()
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
            gad_queue.put((ip,message)) # put in the GAD queue
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
