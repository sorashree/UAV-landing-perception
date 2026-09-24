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
                return None, 0.0
            return self.initialize(frame, measurement_box),1.0

        predicted=self.kf.predict()
        pcx,pcy=float(predicted[0]), float(predicted[1])

        box=None
        tracker_conf=0.0

        if self.tracker is not None:
            try:
                ok,candidate=self.tracker.update(frame)
                if ok:
                    box=tuple(map(int,candidate))
                    tracker_conf=0.78
            except Exception:
                box=None

        if measurement_box is not None:
            mcx, mcy=self.__center(measurement_box)
            jump=math.hypot(mcx-pcx, mcy-pcy)
            max_jump=max(frame.shape[1], frame.shape[0]) *0.18
            if box is None or jump<max_jump:
                box=measurement_box
                tracker_conf=max(tracker_conf, 0.92)

        if box is None:
            self.missed+=1
            if self.last_box is not None and self.missed<=8:
                x,y,w,h=self.last_box
                dx,dy=pcx-self._center(self.last_box)[0], pcy-self._center(self.last_box)[1]
                box=(x+dx, y+dy,w,h)
                box=self._clip_box(box,frame.shap[1], frame.shape[0])
                return box, max(0.25,0.72-self.missed*0.06)
            self.active=False
            return None, 0.0

        self.missed=0
        box=self._clip_box(box,frame.shape[1], frame.shape[0])
        cx,cy=self._center(box)

        self.kf.correct(np.array([[cx], [cy]], np.float32))

        alpha=0.22
        if self.sx is None:
            self.sx, self.sy=cx,cy
        else:
            self.sx=(1-alpha) * self.sx+aplha*cx
            self.sy=(1-aplha) * self.sy+aplha*cy

        x,y,bw,bh=box
        smoothed=(
            int(round(self.sx-bw/2)),
            int(round(self.sy-bh/2)),
            bw,
            bh,
        )
        smoothed=self._clip_box(smoothed, frame.shape[1], frame.shape[0])

        self.last_box=smoothed
        return smoothed, min(0.99, tracker_conf+0.10)

class CandidateDetector:
    def __init__(self):
        self.bg=cv2.createBackgroundSubtractorMOG2(
            history=180, varThreshold=28, detectShadows=False
        )
        self.prev_gray=None
        
    




