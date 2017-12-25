#!/usr/bin/env python

import numpy as np
import scipy.stats
from params import nparticles, u_noise, z_noise
from motion import motion_model
from measure import measurement_model, calc_expected_measurement


class EKF():
    '''Contains all data and methods for implementing an EKF'''

    def __init__(self, initial_pose, initial_variance):
        '''initialize required variables for t=0'''
        self.mu = initial_pose
        self.sigma = initial_variance


    def mu_(self):
        return np.array([self.mu.x, self.mu.y, self.mu.theta])


    def motion_update(self, u):
        '''calculate updated pose & covariance given a control input'''
        # G must be updated before mu (since equation uses mu at t-1)
        G = np.eye(3)
        if u.w != 0:
            G02 = -(u.v/u.w) * np.cos(self.mu.theta) + (u.v/u.w) * np.cos(self.mu.theta + u.w*u.dt)
            G12 = -(u.v/u.w) * np.sin(self.mu.theta) + (u.v/u.w) * np.sin(self.mu.theta + u.w*u.dt)
        else:
            G02 = -u.v * u.dt * np.sin(self.mu.theta)
            G12 =  u.v * u.dt * np.cos(self.mu.theta)
        G[0, 2] = G02
        G[1, 2] = G12

        # 1) update pose estimate (noise=False since it's incorporated in sigma)
        self.mu = motion_model(u, self.mu, add_noise=False)

        # 2) update covariance
        # TODO: make definition of R realistic and move to correct place
        R = np.array([[0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]])
        self.sigma = np.matmul(np.matmul(G, self.sigma), G.T) + R # TODO WTF is R???

        return self.mu, self.sigma


    def measurement_update(self, z, LM):
        '''update weights given measurement data to an observed landmark
        implemented based off Probabilistic Robotics Table 3.3 (page 59)'''

        try: lm = LM[z.s];
        except: return 0; # measurement was to a robot, not a landmark

        H00 = (self.mu.x - lm.x) / np.sqrt((lm.x - self.mu.x)**2 + (lm.y - self.mu.y)**2)
        H01 = (self.mu.y - lm.y) / np.sqrt((lm.x - self.mu.x)**2 + (lm.y - self.mu.y)**2)
        H10 = (lm.y - self.mu.y) / ((lm.x - self.mu.x)**2 + (lm.y - self.mu.y)**2)
        H11 = (lm.x - self.mu.x) / ((lm.x - self.mu.x)**2 + (lm.y - self.mu.y)**2)

        H = np.array([[H00, H01, 0], [H10, H11, -1]])
        print "H:\n", H

        # TODO: measurement covariance, move this to params file
        Q = np.array([[0.1, 0], [0, 0.2]])

        thisequationistoolong = np.linalg.inv(np.dot(np.dot(H, self.sigma), H.T) + Q)
        K = np.dot(np.dot(self.sigma, H.T), thisequationistoolong)
        print "K:\n", K

        zt = np.array([z.r, z.b])
        tmp = calc_expected_measurement(self.mu, lm)
        h_bar = np.array([tmp[0], tmp[1]])
        tmp = self.mu_() + np.dot(K, (zt - h_bar))
        print "mu before:", tmp[0], ", ", tmp[1], ", ", tmp[2]
        self.mu.x, self.mu.y, self.mu.theta = tmp[0], tmp[1], tmp[2]
        print "mu:", self.mu

        self.sigma = np.dot(np.eye(3) - np.dot(K, H), self.sigma)
        print "sigma:\n", self.sigma

        return 1



if __name__ == '__main__':
    print "[WARNING] filter.py should not be run as main"

    # UNIT TESTING
    from definitions import Control, Pose, PoseStamped, MeasurementStamped

    ekf = EKF(PoseStamped(0.0, 0.0, 0.0, 0.0), np.array([[0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]]))
    print "mu:", ekf.mu
    print "sigma:\n", ekf.sigma, "\n"

    # for i in xrange(5):
    #     ekf.motion_update(Control(0.1, 1.0, 1.0))
    #     print("[{}] mu: {}").format(i, ekf.mu)
    #     print "sigma:\n", ekf.sigma

    z = MeasurementStamped(0.0, 99, 1.5, 0.0)
    lm = {}
    lm[99] = Pose(3.0, 0.0, 0.0)
    ekf.measurement_update(z, lm)
