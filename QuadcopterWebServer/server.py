import tornado.ioloop
import tornado.web
import tornado.websocket
import base64
import sys
import os
import asyncio
import cv2
import numpy as np
from collections import deque
import imutils

settings = {
    "static_path": os.path.join(os.path.dirname(__file__), "static")
}

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
orangeLower = (0, 58, 132)
orangeUpper = (255, 255, 193)
pts = deque(maxlen=64)
(dx, dy) = (0, 0)
direction = ""

# Define measurements used for calibration
imageWidth = 240
imageHeight = 160
ballDiameter = 4.445 # Actual ball diameter (cm)
measuredDistance = 20 # distance between the ball and camera used for calibration (cm)
apparentDiameter = 67 # Perceived diameter of the ball (px)
# Calculate the focal length f = d*z/D
focalLength = apparentDiameter * measuredDistance / ballDiameter # (px)


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("joystick.html")


class WSHandler(tornado.websocket.WebSocketHandler):
    connections = set()

    def check_origin(self, origin):
        return True

    def open(self):
        # close if another controller is already connected
        if (len(self.connections) > 0):
            self.close()
        else:
            self.connections.add(self)

    def on_message(self, message):
        [client.write_message(message)
         for client in FlightWSHandler.connections]
        # print(message)

    def on_close(self):
        if self in self.connections:
            self.connections.remove(self)


class CameraWSHandler(tornado.websocket.WebSocketHandler):
    connections = set()

    def check_origin(self, origin):
        return True

    def open(self):
        # close if another controller is already connected
        if (len(self.connections) > 0):
            self.close()
        else:
            self.connections.add(self)

    def on_message(self, imageBytes):
        imageArr = np.frombuffer(imageBytes, dtype=np.uint8)
        frame = cv2.imdecode(imageArr, cv2.IMREAD_COLOR)
        tracked_image = track(frame)
        ret, message = cv2.imencode('.JPEG', tracked_image)
        [client.write_message(base64.b64encode(message))
         for client in WSHandler.connections]

    def on_close(self):
        if self in self.connections:
            self.connections.remove(self)


class FlightWSHandler(tornado.websocket.WebSocketHandler):
    connections = set()

    def check_origin(self, origin):
        return True

    def open(self):
        # close if another controller is already connected
        if (len(self.connections) > 0):
            self.close()
        else:
            self.connections.add(self)

    def on_message(self, message):
        pass

    def on_close(self):
        if self in self.connections:
            self.connections.remove(self)


def track(frame):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    hsv[...,1] = hsv[...,1]*1
    # construct a mask for the color "orange", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # use triangle similarity to determine current distance from camera
            z = int(ballDiameter * focalLength / (radius*2))
            message = "X: " + str(center[0]-120) + " Y: " + str(center[1]-80) + " Z: " + str(z)
            cv2.putText(frame, message, (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)

    # return mask
    return frame


'''
    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        #thickness = int(np.sqrt(20 / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), 2)
'''


def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/websocket", WSHandler),
        (r"/camera", CameraWSHandler),
        (r"/flight-control", FlightWSHandler),
        (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": "./static"}),
    ], **settings)


if __name__ == "__main__":
    # If running on a Windows machine, use WindowsSelectorEventLoopPolicy
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    # Initialize server, listen to port 8888
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
