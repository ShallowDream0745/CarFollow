from __future__ import print_function



class DynamicsConfig(object):
    DYNAMICS_DIM = 2
    ACTION_DIM = 1
    Ts = 0.05      # control signal period
    Np = 6       # predict horizon
    lambda_cbf = 1/23 # lambda in control barrier function
    x_init = [25, 20]
    alpha = 0.1
    beta = 0.1
    CBF = 1         # whether using CBF



