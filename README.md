# GR6-v1

**GR6-v1** is an in-development, real-time robotic control interface built with Python, Flask-SocketIO, and JavaScript. An OXTS xNAV650 it used for the navigation system, simplifying the project substantially. It’s tailored to a custom robot platform and emphasizes modular design, maintainability, and responsiveness.

> ⚠️ This project is designed for a one-off robot and may not run as-is on other hardware.

The repo runs on a Raspberry Pi (RPI 4 in my case, but I have used RPI 3 too).

## Features (so far)

- Picamera2 collecting images
- Decode of OXTS NCOM data to dictionaries
- Webpages showing xNAV650 data and camera images using Flask
- Plumbing for commands to be sent back from the webpage to the RPI

## Status

Currently under development and likely to change often. Use at your own curiosity.

## TODO

The list is long...

- Camera calibration routines - needed so aruco updates work properly
- Map making for aruco markers
- Manual motor control (there's an Arduino on a USB port that controls the motors)
- Wheel encoder feedback to go to the xNAV650 through OXTS GAD update
- Aruco code was taken from a previous project, so it probably doesn't work properly and needs attention
- And then all the robotic stuff - path following, path planning, etc.
