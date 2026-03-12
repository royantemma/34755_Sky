#!/usr/bin/env python3
# Rui Santos & Sara Santos - Random Nerd Tutorials
# Complete project details at https://RandomNerdTutorials.com/raspberry-pi-mjpeg-streaming-web-server-picamera2/

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:7123
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import io
import logging
import socketserver
import socket

import threading
import simplejpeg
import numpy
import cv2 as cv

from http import server
from threading import Condition
from setproctitle import setproctitle
#
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# set title of process, so that it is not just called Python
setproctitle("stream_server")

hostname = socket.gethostname()
# PAGE = """\
# <html>
# <head>
# <title>picamera2 MJPEG</title>
# </head>
# <body>
# <h1>Picamera2 MJPEG Streaming from {}</h1>
# <img src="stream.mjpg" width="820" height="616" />
# <!--img src="stream.mjpg" width="1296" height="972" -->
# <!--img src="stream.mjpg" width="320" height="240" -->
# <!--img src="stream.mjpg" width="640" height="480" -->
# </body>
# </html>
# """.format(hostname)

### MODIFICATION THEO - MULTISTREAMING ###
PAGE = """\
<html>
<head>
<title>Picamera2 Modular MJPEG</title>
</head>
<body>
<h1>Main Camera Stream</h1>
<img src="/stream/main" width="820" height="616">

<h1>Custom Image Stream</h1>
<img src="/stream/cameratest" width="820" height="616">
</body>
</html>
""".format(hostname)
### END MODIFICATION THEO - MULTISTREAMING ###


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


### MODIFICATION THEO - MULTISTREAMING ###
class StreamManager:
    def __init__(self):
        self.streams = {}  # name: {output, enabled}

    def add_stream(self, name):
        """Register a new stream and return its output object."""
        self.streams[name] = {'output': StreamingOutput(), 'enabled': True}
        return self.streams[name]['output']

    def set_enabled(self, name, enabled=True):
        """Enable or disable a stream dynamically."""
        if name in self.streams:
            self.streams[name]['enabled'] = enabled

    def get_output(self, name):
        """Return the output object if enabled, else None."""
        if name in self.streams and self.streams[name]['enabled']:
            return self.streams[name]['output']
        return None

# Global instance to be imported anywhere
stream_manager = StreamManager()
### END MODIFICATION THEO - MULTISTREAMING ###


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        #elif self.path == '/stream.mjpg':
        ### MODIFICATION THEO - MULTISTREAMING ###
        elif self.path.startswith('/stream/'):
            # Get stream name
            stream_name = self.path.split('/')[-1]
            output = stream_manager.get_output(stream_name)
            if not output:
                self.send_error(404)
                return
        ### END MODIFICATION THEO - MULTISTREAMING ###
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


### START MODIFICATION THEO - MULTISTREAMING ### moved this code inside the process_frames function
"""
# # 1296x972
picam2 = Picamera2()
#picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
# higher resolution and lower framerate (5 FPS (200000 microseconds between frames))
picam2.configure(picam2.create_video_configuration(main={"size": (820, 616)},controls={'FrameDurationLimits': (200000, 500000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (1296, 972)},controls={'FrameDurationLimits': (200000, 500000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (1296, 972)},controls={'FrameDurationLimits': (200000, 200000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (1296, 972)},controls={'FrameDurationLimits': (50000, 200000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)},controls={'FrameDurationLimits': (200000, 500000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (320, 240)},controls={'FrameDurationLimits': (200000, 500000)}))

#output = StreamingOutput()
main_output = stream_manager.add_stream("main")
"""
### END MODIFICATION THEO - MULTISTREAMING ###
main_output = stream_manager.add_stream("main")
cameratest_output = stream_manager.add_stream("cameratest")

#picam2.start_recording(JpegEncoder(), FileOutput(output))
### MODIFICATION THEO ### STATUS: working as intended, finished
# This section intends to modify the image before it is send to the life stream.
# All the code in this section is the replacement of the commented line of code just above.

def process_frames():
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (820, 616)},controls={'FrameDurationLimits': (200000, 500000)}))
    picam2.start()
    
    while True:
        frame = picam2.capture_array()

        # Picamera2 XBGR8888 is RGBX in memory order — drop the padding channel.
        if frame.shape[2] == 4:
            frame = numpy.ascontiguousarray(frame[:, :, :3])

        jpeg = simplejpeg.encode_jpeg(frame, quality=80, colorspace='RGB')

        main_output.write(jpeg)

    # Theo process frames in black and white
    # while True:
    #     frame = picam2.capture_array()

    #     # convert to grayscale
    #     gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #     processed = cv.cvtColor(gray, cv.COLOR_GRAY2BGR)

    #     # draw example
    #     cv.putText(processed, "Processed", (30,50),
    #                cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    #     jpeg = simplejpeg.encode_jpeg(processed, quality=80)

    #     main_output.write(jpeg)

#threading.Thread(target=process_frames, daemon=True).start()

### END MODIFICATION THEO ###


# try:
#     address = ('', 7123)
#     server = StreamingServer(address, StreamingHandler)
#     server.serve_forever()
# finally:
#     picam2.stop_recording()

### START MODIFICATION THEO - MULTISTREAMING ###
def start_stream_server():
    threading.Thread(target=process_frames, daemon=True).start()

    address = ('', 7123)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()


if __name__ == "__main__":
    try:
        start_stream_server()
    finally:
        pass
    #     picam2.stop_recording()
### END MODIFICATOIN THEO - MULTISTREAMING ###
