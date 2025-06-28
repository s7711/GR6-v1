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
import datetime
import socket
import sys
import os
import re
import math
import numpy as np
import logging
import subprocess

# Local modules
import web_server
import bgCamera2
import ncomrx
import config
import gad_fake
import gad_aruco
import xnav

logging.info("Program starting") # Makes sure that the default logging handler is set up
CFG = config.ReadConfig() # Read the configuration file
         
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
xn = xnav.XNav(CFG['InsIp']) # xNAV receiver and decoder
nrxs = xn.nrxs # Get the NcomRxThread from the xnav
xn.nrxs.moreCalcs.append(calcAmLocal)     # Add in calculation for AmLocal co-ordinates

cam = bgCamera2.BgCamera() # Opens the camera in the background
ws = web_server.WebServer() # Starts a web server in the background using Flask-SocketIO
xn.ws = ws # Set the web server for the xNAV to send data to
ga = gad_aruco.GadAruco(CFG)
ga.nrxs = xn.nrxs
ga.ws = ws
ga.cam = cam
ws.set_camera(ga.ar.img)          # web server /camera.mjpg will use this image
gf = gad_fake.GadFake(CFG) # Starts the GAD fake thread

#####################################################################
# Start the program
logging.info("Use Ctrl-C to quit")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Socket for sending UDP

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
