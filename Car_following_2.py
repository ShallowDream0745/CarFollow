from casadi import *
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import time

# config:
TMAX = 750
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
    state = np.array([0, -1, 0])  # suppose v_f[0]=v_p[0]
    des_trajectory = []
    state_trajectory = []
    control_trajectory = []
    distance_trajectory = []
    cal_time = 0.0

    for t in range(TMAX-1):
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

        des_trajectory.append(get_des(v_p[t]-state[1]))
        distance_trajectory.append(d)
        state_trajectory.append(state)
        control_trajectory.append(control)
        if is_violating(control, state, v_p):
            print('state:', state)
            print('control:', control)
            print('time:', t)
            return (False,
                    np.array(des_trajectory),
                    np.array(state_trajectory),
                    np.array(control_trajectory),
                    np.array(distance_trajectory),
                    v_p[0:-1])
    print('Calculating time: {:.3f}s'.format(cal_time))
    return (True,
            np.array(des_trajectory),
            np.array(state_trajectory),
            np.array(control_trajectory),
            np.array(distance_trajectory),
            v_p[0:-1])


def myplot(data,
           figure_num=1,
           mode="xy",
           fname=None,
           shownow=False,
           gridon=True,
           title=None,
           xlabel=None,
           ylabel=None,
           legend=None,
           legend_loc="best",
           color_list=None,
           xlim=None,
           ylim=None,
           ncol=1):
    """
    plot figures
    """

    # _, ax = plt.subplots()
    if figure_num == 1:
        data = [data]

    if color_list is not None:
        for (i, d) in enumerate(data):
            if mode == "xy":
                plt.plot(d[0], d[1], color=color_list[i])
            if mode == "y":
                plt.plot(d, color=color_list[i])
            if mode == "scatter":
                plt.scatter(d[0], d[1], color=color_list[i], marker=".", s=5.,)
    else:
        for (i, d) in enumerate(data):
            if mode == "xy":
                plt.plot(d[0], d[1])
            if mode == "y":
                plt.plot(d)
            if mode == "scatter":
                plt.scatter(d[0], d[1], marker=".", s=5.,)

    plt.tick_params(labelsize=10)
    # labels = ax.get_xticklabels() + ax.get_yticklabels()
    # [label.set_fontname('Calibri') for label in labels]
    font = {'family': 'Calibri', 'size': '10'}
    if legend is not None:
        plt.legend(legend, loc=legend_loc, ncol=ncol, prop=font)
        # 'lower center'
    plt.xlabel(xlabel, font)
    plt.ylabel(ylabel, font)
    if xlim is not None:
        plt.xlim(xlim)
    if ylim is not None:
        plt.ylim(ylim)
    plt.tight_layout()
    if title:
        plt.title(title)
    if gridon:
        plt.grid(True)
    if fname is None:
        if shownow:
            plt.show()
    else:
        plt.savefig(fname)


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
        elif opt['method'] == 'CBF':  # using CBF, only second steps
            cbf_lambda = opt['paras']
            if time == 2:
                g += [get_constraint(s_)-(1-cbf_lambda) *
                                     get_constraint(x[2*time])]
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


def generate_vp(mode, tmax):  # generate v_p sequence
    # several modes of v_p
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
    elif mode == 4:
        v_p = np.load('D:\CppAndPython\GitTest\\v_p1.npy')
    # plt.plot(t_sequence*DT,v_p)
    # plt.title('speed of front car: v_p')
    # plt.xlabel('t/s')
    # plt.ylabel('$v_p/(m\cdot s^{-1})$')
    # plt.show()
    return v_p


def plot_result(des, state, control, distance, vp, saving, save_name):

    n = len(state)
    t = np.arange(n) * DT
    data_delta_d = [t, state[:, 0]]
    vp = vp[:n]
    plt.figure(figsize=(15, 8))
    plt.subplot(2, 2, 1)
    myplot(
        [data_delta_d, [t, des]],
        figure_num=2,
        xlabel='time / s',
        ylabel='$\Delta d$ / m',
        legend=['$\Delta d$', '$d_{des}$'],
        title='$\Delta d$'
    )
    plt.subplot(2, 2, 2)
    data_v = [[t, vp], [t, vp - state[:, 1]]]
    myplot(
        data_v,
        figure_num=2,
        legend=['Proceeding Vehicle', 'Self Vehicle'],
        xlabel='time / s',
        ylabel='v / (m/s)',
        title='$Velocity$'
    )
    plt.subplot(2, 2, 3)
    data_d = [[t, distance], [t, D_S0 * np.ones(len(t))]]
    myplot(
        data_d,
        figure_num=2,
        legend=['Distance', 'Constraints'],
        xlabel='time / s',
        ylabel='d / m',
        title='True Distance'
    )
    plt.subplot(2, 2, 4)
    myplot(
        [t, control],
        xlabel='time / s',
        ylabel='$a_{fdes}$ / (m/$s^2$)',
        title='Control Value'
    )
    if saving:
        plt.savefig(save_name)
    # plt.show()
    plt.close()


if __name__ == '__main__':
    opt = {'method': 'CBF',
            'paras': 0.3,
            'optimizer': 'ipopt',
            'vp_mode': 4,
            }
    saving = True
    # safety, des, state, control, distance, v_p = main(opt)
    for method in ['PTW', 'CBF']:
        opt['method'] = method
        for mode in range(1, 5):
            opt['vp_mode'] = mode
            if method == 'CBF':
                for i in range(1, 10, 2):
                    opt['paras'] = float(i)/10
                    save_name = "./GitTest/fig/"+"vp{0}_CBF_{1}.png".format(opt['vp_mode'], opt['paras'])
                    safety, des, state, control, distance, v_p=main(opt)
                    plot_result(des, state, control, distance,
                                v_p, saving, save_name)
            else:
                for i in range(5, 31, 5):
                    NP=i
                    save_name="./GitTest/fig/" +"vp_{0}_PTW_NP_{1}.png".format(opt['vp_mode'], i)
                    safety, des, state, control, distance, v_p=main(opt)
                    plot_result(des, state, control, distance,
                                v_p, saving, save_name)

    # plot_result(des, state, control, distance, v_p,saving,save_name)
    if safety:
        print('Success!')
    else:
        print('Interrupted!')
