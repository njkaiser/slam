#!/usr/bin/env python

import numpy as np
import scipy.stats
from params import nparticles, u_noise, z_noise
from motion import motion_model
from measure import measurement_model


class EKF():
    '''Contains all data and methods for implementing an EKF'''

    def __init__(self, initial_pose, initial_variance):
        '''initialize required variables for t=0'''
        self.mu = initial_pose
        self.sigma = initial_variance


    def motion_update(self, u):
        '''calculate updated pose & covariance given a control input'''
        # G must be updated before mu (since equation uses mu at t-1)
        g02 = -(u.v/u.w) * np.cos(self.mu.theta) + (u.v/u.w) * np.cos(self.mu.theta + u.w*u.dt)
        g12 = -(u.v/u.w) * np.sin(self.mu.theta) + (u.v/u.w) * np.sin(self.mu.theta + u.w*u.dt)
        G = np.eye(3)
        G[0, 2] = g02
        G[1, 2] = g12

        # 1) update pose estimate (noise=False since it's incorporated in sigma)
        self.mu = motion_model(u, self.mu, add_noise=False)

        # 2) update covariance
        # TODO: make definition of R realistic and move to correct place
        R = np.array([[0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]])
        self.sigma = np.matmul(np.matmul(G, self.sigma), G.T) + R # TODO WTF is R???

        return self.mu, self.sigma


    def measurement_update(self, z, LM):
        '''update weights given measurement data to an observed landmark'''

        return 1


if __name__ == '__main__':
    print "[WARNING] filter.py should not be run as main"

    # UNIT TESTING
    from definitions import Control, PoseStamped
    ekf = EKF(PoseStamped(0.0, 0.0, 0.0, 0.0), np.array([[0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]]))

    print "mu:", ekf.mu
    print "sigma:\n", ekf.sigma
    for i in xrange(5):
        ekf.motion_update(Control(0.1, 0.0, 1.00000000))
        print "[ i ] mu: [", i, "]", ekf.mu
        print "sigma:\n", ekf.sigma
