#!/usr/bin/env python
import numpy as np
import scipy.linalg
import my_util


class Radar_model:

    def __init__(self, radar_dt):

        self.F = np.array([[1., 0., radar_dt, 0.],
                           [0., 1., 0., radar_dt],
                           [0., 0., 1, 0.],
                           [0., 0., 0, 1.]])

        self.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])

        G = np.array([[radar_dt**2 / 2., 0.],
                      [0., radar_dt**2 / 2.],
                      [radar_dt, 0.],
                      [0., radar_dt]])

        self.Q = G.dot(np.diag([1.5**2, 1.5**2])).dot(G.T)

        self.R = np.diag([1.**2, 1.**2])

    def predict(self, x, P):
        x = self.F.dot(x)
        P = self.F.dot(P).dot(self.F.T) + self.Q
        return x, P

    def update(self, x, P, z):
        y = z - self.H.dot(x)
        S = self.H.dot(P).dot(self.H.T) + self.R
        K = P.dot(self.H.T).dot(np.linalg.inv(S))
        x = x + K.dot(y)
        P = (np.eye(4) - K.dot(self.H)).dot(P)
        return x, P
