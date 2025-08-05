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
import numpy as np

# Local modules
import web_server
import bgCamera2
from config import CFG
import config
import gad_fake
import gad_aruco
import gad_wheelspeed
import xnav
import motors
import sys_stat
import maps
import path_follow

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
gf.ws = ws                                          # Set the web server for the GAD fake to send data to

mc = motors.MotorController()
mc.ws = ws

gw = gad_wheelspeed.GadWheelspeed()                 # GadWheelspeed sends updates from motor
gw.ws = ws
gw.nrxs = xn.nrxs
mc.gad_wheelspeed = gw

ss = sys_stat.sysStat()                             # Gathers and sends system information
ss.ws = ws

#{map_name: (creator, {channels:<n>, fill:<value>, dtype:<dtype>})}
# 250704 - maps is not working yet. This might have to be replaced with a
# different scheme.
# TODO: fix maps
map_list = {
    "ground_rgb": ("create_ground_rgb", {"grid_spacing_m": 5.0}),
    "wifi_strength": ("create_map", {"channels": 1, "fill": 0, "dtype": np.uint8}),
    "ground_height": ("create_map", {"channels": 1, "fill": 100.0, "dtype": np.float32})
}
mp = maps.Maps(map_list)
xn.nrxs.moreCalcs.append(maps.calcMapLocal)     # Add in calculation for MapLocal co-ordinates: XYZ

pf = path_follow.PathFollow()
pf.ws = ws
pf.motor = mc
pf.nrx = xn.nrxs.nrx[CFG['InsIp']]['decoder']

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
            subprocess.call('./shareWlan0Eth0')
            continue
        elif message.startswith('!'):
            xn.user_command(message)     
        elif message.startswith('#'): # then assume it is a GAD message
            gf.user_command(message) # Use the GAD fake thread
        elif message.startswith('&'):
            mc.user_command(message)
        elif message.startswith('*'):
            config.user_command(message)
        elif message.startswith('>'):
            pf.user_command(message)
        elif message.startswith('<'):
            gw.user_command(message)
        elif message.startswith(':'):
            xn.nrxs.user_command(message)
        elif message.startswith("{"):
            ga.user_command(message)

except KeyboardInterrupt as e:
    print('Stopping')
    mp.save_all()
    # Needs extra code to stop threads, which may be blocked on sockets
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(1)
