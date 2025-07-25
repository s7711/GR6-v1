# GR6-v1

**GR6-v1** is an in-development, real-time robotic control interface built with Python, Flask-SocketIO, and JavaScript. An OXTS xNAV650 is used for the navigation system, simplifying the project substantially. It’s tailored to a custom robot platform and generally does a lot of work from scratch, rather than using ROS or another robot api.

> ⚠️ This project is designed for a one-off robot and may not run as-is on other hardware.

The repo runs on a Raspberry Pi (RPI 4 in my case, but I have used RPI 3 too).

## Features (so far)

- Picamera2 collecting images
- Decode of OXTS NCOM data to dictionaries
- Webpages showing xNAV650 data and camera images using Flask
- Plumbing for commands to be sent back from the webpage to the RPI
- Manual driving of the motor from a webpage
- Creating a path to follow by driving it and setting waypoints
- Following a path automatically

## Status

Currently under development and likely to change often. Use at your own curiosity. The code is more for example than for use by others.

## TODO

The list is long...

- Camera calibration routines - needed so aruco updates work properly
- Map making for aruco markers
- Using aruco markers from a map to improve navigation
- Connecting up and using ultrasonic distance sensors
- Fixing problems with OXTS GAD, because it seems to work intermittently
- Redoing the web pages, because they are all hacked together quickly
- Maybe replacing Flask, because it is not working very well in my real-time, online application
- Lots of other TODO's left in the code, and things that need improving which don't have TODO's by them
