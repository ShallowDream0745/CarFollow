from casadi import *
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import time
import csv
from utils import *

# config:
TMAX = 750
DYNAMICS_NUM = 3
ACTIONS_NUM = 1

DT = 0.1  # time intervals

TAU_H = 1  # (s^-1)
R = 0.054  # (s^2 m^-1)
V_F_MEAN = 17.5  # (m/s)
D_0 = 2.9  # (m)
D_S0 = 5  # (m)
T_G = 0.393  # (s)
K_G = 1.05
TTC = -2.5  # (s)




delta_d_max = np.inf
delta_d_min = -np.inf
delta_v_max = 15
delta_v_min = -15
a_f_max = np.inf
a_f_min = -np.inf
control_max = 1.5
control_min = -1.5
state_min = np.array([delta_d_min, delta_v_min, a_f_min])
state_max = np.array([delta_d_max, delta_v_max, a_f_max])


def main(opt):
    v_p = generate_vp(opt['vp_mode'], TMAX)
    state = np.array([0, 0, 0])  # suppose v_f[0]=v_p[0]
    des_trajectory = []
    state_trajectory = []
    control_trajectory = []
    distance_trajectory = []
    time_trajectory = []
    cost_trajectory = []
    cal_time = 0.0
    a_p = (v_p[1] - v_p[0]) / DT

    for t in range(TMAX-1):
        start_time = time.time()
        # get the control value and the next state
        # the fourth element is the control
        variants, cost = mpcSolver(state, v_p[t], a_p,  opt)
        control = variants[3]
        time_trajectory.append(time.time() - start_time)
        cost_trajectory.append(cost[0])
        # difference for a_p
        a_p = (v_p[t + 1] - v_p[t]) / DT

        # get real distance between
        v_f = v_p[t] - state[1]
        d_des = R * v_f * (v_f - V_F_MEAN) + TAU_H * v_f + D_0
        d = d_des + state[0]

        # update state
        state = np.array([
            state[0]+DT*state[1]-DT*get_diffdes(v_p[t])*state[2],
            state[1]-DT*state[2] + DT * a_p,
            (1-DT/T_G)*state[2]+K_G/T_G*DT*control
        ])

        des_trajectory.append(get_des(v_p[t]-state[1]))
        distance_trajectory.append(d)
        state_trajectory.append(state)
        control_trajectory.append(control)
    # print('Calculating time: {:.3f}s'.format(cal_time))

    return (np.array(des_trajectory),
            np.array(state_trajectory),
            np.array(control_trajectory),
            np.array(distance_trajectory),
            np.array(cost_trajectory),
            np.array(time_trajectory),
            v_p[0:-1])



if __name__ == '__main__':
    opt = {'method': 'CBF',
            'paras': 0.01,
            'NPC': 30,
            'NP': 50,
            'optimizer': 'ipopt',
            'vp_mode': 5,
            'obs':False
            }

    for method in ['CBF','PTW','TP']:
        for NP in [200]:
            for optimizer in ['ipopt']:
                if method == 'PTW':
                    for NPC in [10]:
                        opt['method'] = method
                        opt['NP']= NP
                        opt['NPC'] = NPC
                        opt['optimizer'] = optimizer
                        des, state, control, distance, cost, cal_time, v_p = main(opt)
                        save_file(des, state, control, distance, cost, cal_time, v_p, opt)
                if method == 'CBF':
                    for paras in [0.001]:
                        opt['method'] = method
                        opt['NP'] = NP
                        opt['paras'] = paras
                        opt['optimizer'] = optimizer
                        des, state, control, distance, cost, cal_time, v_p = main(opt)
                        save_file(des, state, control, distance, cost, cal_time, v_p, opt)
                if method == 'TP':
                    for paras in [0.005]:
                        opt['method'] = method
                        opt['NP'] = NP
                        opt['paras'] = paras
                        opt['optimizer'] = optimizer
                        des, state, control, distance, cost, cal_time, v_p = main(opt)
                        save_file(des, state, control, distance, cost, cal_time, v_p, opt)




