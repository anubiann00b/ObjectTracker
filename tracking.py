from collections import deque
from multiprocessing import Process, Pipe
import numpy as np
import argparse
import imutils
import cv2
import time
import math
import win32api

# Based on http://www.pyimagesearch.com/2015/09/21/opencv-track-object-movement/

def runCamera(conn):
    colorLower = (20, 100, 100)
    colorUpper = (40, 255, 255)

    bufferLen = 32
    pts = deque(maxlen=bufferLen)
    counter = 0
    startTime = 0

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FPS, 60)

    while True:
        startTime = time.time()
        # print "time: {0}".format(1/(time.time() - startTime))

        (grabbed, frame) = camera.read()
        print "frameTime: {0}".format(time.time() - startTime)
        frame = cv2.flip(frame, 1)
        height, width, channels = frame.shape

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        print "maskTime: {0}".format(time.time() - startTime)
     
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        print "cntTime: {0}".format(time.time() - startTime)

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
     
            # cull out noise
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                pts.appendleft(center)

                px, py = center
                dx = px - width/2
                dy = py - height/2
                conn.send((dx, dy))

        # loop over the set of tracked points
        for i in np.arange(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(bufferLen / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
     
        cv2.line(frame, (width/2-10, height/2), (width/2+10, height/2), (0, 255, 0), 2)
        cv2.line(frame, (width/2, height/2-10), (width/2, height/2+10), (0, 255, 0), 2)
     
        # cv2.imshow("Frame", frame)
        # key = cv2.waitKey(0) & 0xFF
        counter += 1
     
        # if key == ord("q"):
            # break
     
    camera.release()
    cv2.destroyAllWindows()

def updateCursor(conn):
    dx, dy = 0, 0
    scale = 0.01
    while True:
        if conn.poll():
            dx, dy = conn.recv()
            if (abs(dx) < 20): dx = 0
            if (abs(dy) < 20): dy = 0
        mx, my = win32api.GetCursorPos()
        win32api.SetCursorPos((mx + int((dx+math.copysign(99,dx)) * scale), my + int((dy+math.copysign(99,dy)) * scale)))
        time.sleep(0.01)

if __name__ == '__main__':
    parent_conn, child_conn = Pipe()
    p = Process(target=updateCursor, args=(child_conn,))
    p.daemon = True
    p.start()

    runCamera(parent_conn)