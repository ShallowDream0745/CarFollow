import numpy as np
import time
from utils import generate_vp, mpcSolver, get_diffdes, is_violating, evaluation, plot_result

# config:
tmax = 600
DYNAMICS_NUM = 3
ACTIONS_NUM = 1

DT = 0.1  # time intervals
NP = 15  # prediction steps

TAU_H = 1.68  # (s^-1)
R = 0.054  # (s^2 m^-1)
V_F_MEAN = 12.5  # (m/s)
D_0 = 2.9  # (m)
D_S0 = 5  # (m)
T_G = 0.393  # (s)
K_G = 1.05
TTC = -2.5  # (s)


def main(opt):
    v_p = generate_vp(opt['vp_mode'], tmax)
    state = np.array([0, -5, 0])  # suppose v_f[0]=v_p[0]
    state_trajectory = []
    control_trajectory = []
    distance_trajectory = []
    cal_time = 0.0

    for t in range(tmax-1):
        start_time = time.time()
        # get the control value and the next state
        # the fourth element is the control
        control = mpcSolver(state, v_p[t], opt)[3]
        cal_time += time.time() - start_time

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
        distance_trajectory.append(d)
        state_trajectory.append(state)
        control_trajectory.append(control)
        if is_violating(control, state, v_p):
            print(state)
            print(control)
            return (False, np.array(state_trajectory), np.array(control_trajectory), np.array(distance_trajectory), v_p[0:-1])
    print('Calculating time: {:.3f}s'.format(cal_time))
    return True, np.array(state_trajectory), np.array(control_trajectory), np.array(distance_trajectory), v_p[0:-1]

if __name__ == '__main__':
    opt = {'method':'PTW', 'paras': 0.3, 'optimizer': 'ipopt', 'vp_mode': 3}
    _, state, control, distance, v_p = main(opt)
    delta_d, delta_v = evaluation(state, control)
    print(delta_d, delta_v)
    plot_result(state, control, distance, v_p)
    a = 1
