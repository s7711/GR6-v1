# config.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Configuration file for GR6-v1

The configuration file is a JSON file and is read into a dictionary.
Usage:
   from config import CFG
Then CFG will exist in the scope of the file you are importing CFG into.

Note: CFG was originally passed into several classes, but it probably
makes more sense to just import it now (not likely to run several
instances with different configurations). You may find a mixture
in the code.
"""
import os
import json
import logging

DEFAULT_CFG = {
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

# This is where CFG is filled in
# Python knows to run this code once, even if it is imported
# many times
CFG = DEFAULT_CFG.copy()  # Start with the default configuration
# Read configuration files, if one exists
try:
    cfg1 = {}
    homeDir = os.path.expanduser('~')
    with open(os.path.join(homeDir,'.GR6_configuration.json')) as f:
        cfg1 = json.load(f)
    for key in cfg1:
        CFG[key] = cfg1[key]
except Exception as e:
    logging.exception(e)
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
    logging.exception(e)

# Can be used to set or change configurations from user_commands
def user_command(message):
    if message.startswith("*"):
        args = message[1:].split()
        if len(args) == 2:
            CFG[args[0]] = args[1]
            logging.info(f"[Config]: {args[0]} = {args[1]}")
        else:
            logging.warning(f"[Config] Bad configuration: {message}")

def getCfgAsInt(key,default):
    try:
        return int(CFG[key])
    except:
        return default