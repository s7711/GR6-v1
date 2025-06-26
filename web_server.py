# web_server.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Implements a background web server using Flask and Flask-SocketIO
Has the following features:
- Serves files from subdirectory "static"
- Defaults "/" to "static/index.html"
- Provides an MJPEG stream at "/camera.mjpg" for the latest camera frame
- Provides a WebSocket endpoint for telemetry updates
- Captures logging messages and sends them through telemetry

Usage:
  ws = WebServer(port=8000)
  ws.send(root, data) - Sends {root:data} as JSON
  ws.send_camera(image_array) - Sends a numpy array image as a JPEG frame
"""

import threading
import flask
import flask_socketio
import cv2
import json
import logging
import collections
import queue
import re
import time

class WebServer:
    def __init__(self, port=8000):
        self.app = flask.Flask(__name__, static_folder="static", static_url_path="")
        self.socketio = flask_socketio.SocketIO(self.app, async_mode="threading")
        self.port = port
        self._camera_function = None  # Function to call for a new frame

        # Flask routes
        self.app.add_url_rule("/", "index", self.index)
        self.app.add_url_rule("/camera.mjpg", "camera_feed", self.camera_feed)

        # WebSocket events
        self.socketio.on_event("connect", self.on_connect)
        self.socketio.on_event("*", self.handle_user_command)

        # Attach logging so logging messages are sent over WebSocket
        self.log_queue = collections.deque(maxlen=20)  # Store last log messages
        websocket_handler = WebSocketLogHandler(self)
        logging.getLogger().addHandler(websocket_handler)  # Add to global logging
        logging.getLogger().setLevel(logging.INFO)  # Ensure logs are captured
        self.ansi_escape = re.compile(r'\x1B\[[0-9;]*[mK]') # To strip ANSI escape codes

        # Queue incomming messages
        self.recv_queue = queue.Queue(maxsize=10)

        # Start server in background thread
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def index(self):
        return flask.send_from_directory("static", "index.html")

    def camera_feed(self):
        def generate():
            while True:
                if self._camera_function is not None:
                    image_array = self._camera_function()
                    if image_array is not None:
                        ret, jpeg = cv2.imencode('.jpg', image_array, [cv2.IMWRITE_JPEG_QUALITY, 30])
                        if ret:
                            yield (b"--frame\r\n"
                                   b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
                time.sleep(0.1)  # Avoid busy loop if no camera
        return flask.Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    def on_connect(self):
        """Send recent log messages when a client connects."""
        logging.info("WebSocket client connected.")
        for message in list(self.log_queue):
            self.socketio.emit("log", message)  # Send to clients

    def send(self, data_type, data):
        """Send data as JSON to all WebSocket clients."""
        # datetime, numpy, etc, need to be converted to string
        # which Flask-SocketIO doesn't do
        try:
            clean_data = json.dumps(data, default=str)  # Ensure proper JSON formatting
            self.socketio.emit(data_type, clean_data)  # Emit event over WebSocket
        except Exception as e:
            logging.error(f"Error sending WebSocket event '{data_type}': {e}")

    def handle_user_command(self,data,event):
        """Receive command from web page and enqueue it."""
        try:
            self.recv_queue.put((event, data), block=False)
        except queue.Full:
            logging.warning(f"Dropped message: {event=}, {data=}")

    def recv(self):
        """Wait for a message from the WebSocket and return it."""
        return self.recv_queue.get()  # Block until a message is received

    def queue_log(self, message):
        """Store the latest log message in the queue."""
        # Some messages have ansi escape codes, which we strip
        clean_message = self.ansi_escape.sub('', message)

        self.log_queue.append(clean_message)
        self.socketio.emit("log", clean_message)  # Send to clients

    def set_camera(self, camera_function):
        """Set the function to call to get a new camera frame (numpy array)."""
        self._camera_function = camera_function

    def _run(self):
        self.socketio.run(self.app, host="0.0.0.0", port=self.port, debug=False)

# Logging handler captures logging messages and sends them to the WebSocket clients
# There's a queue so previous messages are sent when a client connects
class WebSocketLogHandler(logging.Handler):
    def __init__(self, web_server):
        super().__init__()
        self.web_server = web_server

    def emit(self, record):
        """Format and store log messages."""
        log_entry = self.format(record)
        self.web_server.queue_log(log_entry)  # Store in queue
