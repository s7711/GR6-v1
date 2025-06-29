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
import sys
import os
import logging
import subprocess

# Local modules
import web_server
import bgCamera2
from config import CFG
import gad_fake
import gad_aruco
import xnav
import motors

logging.info("Program starting") # Makes sure that the default logging handler is set up
         
# Start up the services
cam = bgCamera2.BgCamera()                          # Opens the camera in the background
ws = web_server.WebServer()                         # Starts a web server in the background

xn = xnav.XNav(CFG['InsIp'])                        # xNAV receiver and decoder
xn.nrxs.moreCalcs.append(gad_aruco.calcAmLocal)     # Add in calculation for AmLocal co-ordinates
xn.ws = ws                                          # Set the web server for the xNAV to send data to

ga = gad_aruco.GadAruco(CFG)
ga.nrxs = xn.nrxs
ga.ws = ws
ga.cam = cam
ws.set_camera(ga.ar.img)                            # web server /camera.mjpg will use this image

gf = gad_fake.GadFake(CFG)                          # Starts the GAD fake thread

mc = motors.MotorController()
mc.ws = ws

#####################################################################
# Start the program
logging.info("Use Ctrl-C to quit")

# Messages are categoried by how they start
# $ - dealt with here
# ! - send to OXTS xnav
# # - gad_fake


try:
    # receive messages from web sockets
    while True:
        message,path = ws.recv() # Note: blocking
        logging.info(path + ": " + message)
        if message == '$shutdown':  # Shutdown RaspPi
            subprocess.run('sudo shutdown now', shell=True)
            continue
        elif message == '$internet-passthrough':
            subprocess.call('./shareWlan1Eth0')
            continue
        elif message.startswith('!'):
            xn.user_command(message)     
        elif message.startswith('#'): # then assume it is a GAD message
            gf.user_command(message) # Use the GAD fake thread
        elif message.startswith('&'):
            mc.user_command(message)

except KeyboardInterrupt as e:
    print('Stopping')
    # Needs extra code to stop threads, which may be blocked on sockets
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(1)
