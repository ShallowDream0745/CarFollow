from casadi import *
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from myplot import myplot

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

W_D = 0.020
W_V = 0.025
W_A = 5.000


def get_des(x): return R*x*(x-V_F_MEAN)+TAU_H*x+D_0
def get_diffdes(x): return TAU_H+R*(2*x-V_F_MEAN)


delta_d_max = np.inf
delta_d_min = -np.inf
delta_v_max = 10
delta_v_min = -10
a_f_max = np.inf
a_f_min = -np.inf
control_max = 1.5
control_min = -1.5
state_min = np.array([delta_d_min, delta_v_min, a_f_min])
state_max = np.array([delta_d_max, delta_v_max, a_f_max])


def is_violating(control, state, v_p):
    if (state[0] > state_max[0] or
        state[1] > state_max[1] or
        state[2] > state_max[2] or
        state[0] < state_min[0] or
        state[1] < state_min[1] or
        state[2] < state_min[2] or
        control > control_max or
            control < control_min):
        return True
    return False


def mpcSolver(state, v_p, opt):
    '''
    state: np.array([d-des,v_p-v_f,a_f])
    v_p: speed of the front car
    des: expected distance between the car
    a_f: acceleration
    u:   control variable
    g:   constraint variable

    RETURN:
    control: (int)
    '''

    v_f = v_p - state[1]
    state_list = state.tolist()
    zero_list = np.zeros(DYNAMICS_NUM).tolist()

    control = MX.sym('x', ACTIONS_NUM)  # 1*1
    s = MX.sym('s', DYNAMICS_NUM)  # 1*3
    s_next = vertcat(
        s[0]+DT*s[1]-DT*get_diffdes(v_f)*s[2],
        s[1]-DT*s[2],
        (1-DT/T_G)*s[2]+K_G/T_G*DT*control
    )
    get_next = Function('f', [s, control], [s_next])

    loss = W_D*s[0]*s[0]+W_V*s[1]*s[1]+W_A*control*control  # 1*1
    get_loss = Function('L', [s, control], [loss])

    constraint = -s[0]+(R*(v_p-s[1]-V_F_MEAN)+TAU_H)*s[1]\
        - TAU_H*v_p+D_S0-D_0
    get_constraint = Function('H', [s], [constraint])

    # independent variable
    x = [MX.sym('s0', DYNAMICS_NUM)]
    x_min = state_list.copy()
    x_max = state_list.copy()
    # constraint variable
    g = []
    g_min = []
    g_max = []
    # loss function
    L = 0

    for time in range(NP):  # time:[0,NP-1]
        # control
        u = MX.sym('u'+str(time))  # 1*1
        x += [u]
        x_min += [control_min]
        x_max += [control_max]
        # state
        s_ = MX.sym('s'+str(time+1), DYNAMICS_NUM)  # 1*3
        x += [s_]
        x_min += state_min.tolist()
        x_max += state_max.tolist()
        # dynamic constraint
        g += [s_-get_next(x[2*time], u)]
        g_max += zero_list
        g_min += zero_list
        # designed constraint
        if opt['method'] == 'PTW':  # Pointwise
            g += [get_constraint(s_)]
            g_max += [0]
            g_min += [-np.inf]
        elif opt['method'] == 'CBF': # using CBF, only second steps
            cbf_lambda = opt['paras']
            if time == 2:
                g += [get_constraint(s_)-(1-cbf_lambda)*get_constraint(x[2*time])]
                g_max += [0]
                g_min += [-np.inf]
        # loss
        L = L+get_loss(s_, u)

    solve_options = {'ipopt.print_level': 0,
                     'ipopt.sb': 'yes', 'print_time': 0}
    solve_instance = {'x': vertcat(*x), 'f': L, 'g': vertcat(*g)}
    solve_function = nlpsol('Func', 'ipopt', solve_instance, solve_options)
    # test
    x_initial = [*state_list, 0]*NP
    x_initial += state_list
    control_sequence = solve_function(
        x0=x_initial, ubx=x_max, lbx=x_min, lbg=g_min, ubg=g_max)
    return np.array(control_sequence['x']).reshape(-1)


def generate_vp(mode,tmax):  # generate v_p sequence
    # mode = 2  # several modes of v_p
    t_sequence = np.arange(tmax)
    if mode == 1:
        half = math.floor(tmax/2)
        v_p1 = 10+5*np.sin(t_sequence[: half]*DT*np.pi/10)
        v_p2 = np.ones(tmax-half)*v_p1[-1]
        v_p = np.r_[v_p1, v_p2]
    elif mode == 2:
        thirdpoint1 = math.floor(tmax/3)
        thirdpoint2 = math.floor(tmax*2/3)
        v_p1 = np.ones(thirdpoint1)*10
        v_p2 = np.linspace(10, 15, thirdpoint2-thirdpoint1)
        v_p3 = np.ones(tmax-thirdpoint2)*v_p2[-1]
        v_p = np.r_[v_p1, v_p2, v_p3]
    elif mode == 3:
        thirdpoint1 = math.floor(tmax/3)
        thirdpoint2 = math.floor(tmax*2/3)
        v_p1 = np.ones(thirdpoint1)*15
        v_p2 = np.linspace(15, 10, thirdpoint2-thirdpoint1)
        v_p3 = np.ones(tmax-thirdpoint2)*v_p2[-1]
        v_p = np.r_[v_p1, v_p2, v_p3]
    plt.plot(t_sequence*DT,v_p)
    plt.title('speed of front car: v_p')
    plt.xlabel('t/s')
    plt.ylabel('$v_p/(m\cdot s^{-1})$')
    plt.show()
    return v_p


def plot_result(des, state, control, distance, vp):

    n = len(state)
    t = np.arange(n) * DT
    data_delta_d = [t, state[:,0]]
    plt.figure(figsize=(15,8))
    plt.subplot(2,2,1)
    myplot(
        [data_delta_d,[t,des]],
        figure_num=2,
        xlabel='time / s',
        ylabel='$\Delta d$ / m',
        legend=['$\Delta d$','$d_{des}$'],
        title='$\Delta d$'
    )
    plt.subplot(2,2,2)
    data_v = [[t, vp], [t, vp - state[:,1]]]
    myplot(
        data_v,
        figure_num=2,
        legend=['Proceeding Vehicle','Self Vehicle'],
        xlabel='time / s',
        ylabel='v / (m/s)',
        title='$Velocity$'
    )
    plt.subplot(2,2,3)
    data_d = [[t, distance], [t, D_S0 * np.ones(len(t))]]
    myplot(
        data_d,
        figure_num=2,
        legend=['Distance', 'Constraints'],
        xlabel='time / s',
        ylabel='d / m',
        title='$True distance$'
    )
    plt.subplot(2,2,4)
    myplot(
        [t, control],
        xlabel='time / s',
        ylabel='$a_{fdes}$ / (m/$s^2$)',
        title='$Control Value$'
    )

    plt.show()

def evaluation(state):
    mean_delta_d = np.mean(np.abs(state[:,0]))
    mean_delta_v = np.mean(np.abs(state[:,1]))
    return mean_delta_d, mean_delta_v

if __name__ == '__main__':
    pass
    # IS_USING_CBF = False
    # cbf_lambda = 0.2
    # max_time = 600
    # i = 0
    # result = main_loop(max_time)
    # print(result[0])
    # plot_result(result[1])
    # plot_result(result[2])
