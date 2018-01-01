#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def PathTrace(data, plotname, hold, color, label):
    plt.plot([a.x for a in data], [a.y for a in data], color=color, label=label)
    if not hold:
        plt.title(plotname)
        plt.xlabel('x position [m]')
        plt.ylabel('y position [m]')
        plt.legend()
        plt.show()


def plot_covariances(fig, ax, poses, covariances):
    '''plot covariances as ellipses'''
    fig, ax = fig, ax # TODO do I need this?


    ells = []
    # TODO: figure out how angle is determined instead of just setting to zero
    for i in xrange(len(poses)):
        ells.append(Ellipse(xy=(poses[i].x, poses[i].y), width=covariances[i][0,0], height=covariances[i][1,1], angle=0))

    for e in ells:
        ax.add_artist(e)
        e.set_clip_box(ax.bbox)
        e.set_alpha(0.5)
        e.set_facecolor(np.random.rand(3))


    # plt.xlim(0.9, 4)
    # # plt.xticks(np.arange(1.0, 4.0 + 1))
    # plt.ylim(-4.8, 1.2)
    # plt.yticks(np.arange(-5.0, 0.0 + 1))
    plt.gca().set_aspect('equal', adjustable='box')

    plt.draw()
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    # plt.show()

    return fig, ax


if __name__ == '__main__':
    print "[WARNING] plot.py should not be run as main"

    # UNIT TESTING:
    from definitions import PoseStamped, ControlStamped
    from motion import motion_model
    from filter import EKF

    covariances = []
    poses = []

    ekf = EKF(PoseStamped(0, 0, 0, 0), np.array([[0.005, 0.0, 0.0], [0.0, 0.005, 0.0], [0.0, 0.0, 0.01]]))
    u = ControlStamped(1, 0, 0.1, 0.1)
    for i in xrange(5):
        print ekf.sigma
        covariances.append(ekf.sigma)
        poses.append(ekf.mu)
        ekf.motion_update(u)

    # print covariances
    # print poses

    fig, ax = plt.subplots()
    PathTrace(poses, 'blah', 1, 'b', 'stuff')
    plot_covariances(fig, ax, poses, covariances)
    plt.show()
