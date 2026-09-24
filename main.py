import argparse
import math
import os
from pathlib import Path

import cv2
import numpy as np

#Drone Landing Perception Demo
class SmoothTracker:
    def __init__(self, fps=30.0):
        self.fps=max(float(fps),1.0)
        self.tracker=None
        self.active=False
        self.last_box=None
        self.last_gray=None
        self.missed=0

        self.kf=cv2.KalmanFilter(4,2)
        self.kf.measurementMatrix=np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix=np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.processNoiseCov=np.eye(4,dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov=np.eye(2, dtype=np.float32)*4.0
        self.errorCovPost=np.eye(4,dtype=np.float32)

        self.sx=None
        self.sy=None

    @staticmethod
    def _center(box):
        x,y,w,h=box
        return np.array([x+w/2.0, y+h/2.0], np.float32)

    @staticmethod
    def _clip_box(box,width,height):
        x,y,w,h=[int(round(v)) for v in box]
        w=max(8,min(w,width-1))
        h=max(8,min(h,height-1))
        x=max(0,min(x,width-w))
        y=max(0,min(y,height-h))
        return (x,y,w,h)

    def _make_tracker(self):
        constructor=[
            lambda: cv2.TrackerCSRT_create(),
            lambda: cv2.legacy.TrackerCSRT_create(),
        ]
        for fn in constructor:
            try:
                return fn()
            except Exception:
                pass
        return None

    def initialize(self, frame, box):
        h,w=frame.shape[:2]
        box=self._clip_box(box,w,h)
        self.last_box=box
        c=self._center(box)

        self.kf.statePost=np.array(
            [c[0],c[1],0,0],np.float32
        ).reshape(4,1)
        self.kf.statePre=self.kf.statePost.copy()
        self.sx,self.sy=float(c[0]), float(c[1])

        self.tracker=self._make_tracker()
        if self.tracker is not None:
            try:
                self.tracker.init(frame,box)
            except Exception:
                self.tracker=None
        self.active=True
        self.missed=0
        self.last_gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return box

    def update(self, frame, measurement_box=None):
        if not self.active:
            if measurement_box is None:

