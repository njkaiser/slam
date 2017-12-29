#!/usr/bin/env python


### FILENAMES:
dataset = 'ds1' # choices are ds0 or ds1
odometry_file_name = dataset + '/' + dataset + '_Odometry.dat'
measurement_file_name = dataset + '/' + dataset + '_Measurement.dat'
groundtruth_file_name = dataset + '/' + dataset + '_Groundtruth.dat'
landmark_file_name = dataset + '/' + dataset + '_Landmark_Groundtruth.dat'
barcode_file_name = dataset + '/' + dataset + '_Barcodes.dat'


### MAIN PROGRAM OPTIONS:
i0 = 0
N = 2500 # clip data for quicker development


### MOTION MODEL PARAMETERS:
class u_noise(object):
    x_rel = 0.150 # x noise (relative) in percent (0.01 = 1%)
    x_abs = 0.005 # x noise (absolute) in meters
    y_rel = 0.001 # y noise should mostly come from theta noise, since robot can't move sideways
    y_abs = 0.005 # y noise (absolute) in meters
    theta_rel = 0.200 # heading noise (relative) in percent (0.01 = 1%)
    theta_abs = 0.020 # heading noise (absolute) in radians


### MEASUREMENT MODEL PARAMETERS:
class z_noise(object):
    r_rel = 0.10 # range noise (relative) in percent (0.01 = 1%)
    r_abs = 0.05 # range noise (absolute) in meters
    # b_rel = 0.10 # doesn't make sense to have this, measurement angle should not affect accuracy
    b_abs = 0.20 # bearing noise - unitless since using cosine comparison



if __name__ == '__main__':
    print "[WARNING] params.py should not be run as main"
