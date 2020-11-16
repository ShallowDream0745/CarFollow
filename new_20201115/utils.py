from casadi import *
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import csv
import pandas as pd

# config:
TMAX = 750
DYNAMICS_NUM = 3
ACTIONS_NUM = 1

DT = 0.1  # time intervals
NP = 30  # prediction steps

TAU_H = 1  # (s^-1)
R = 0.054  # (s^2 m^-1)
V_F_MEAN = 17.5  # (m/s)
D_0 = 2.9  # (m)
D_S0 = 5  # (m)
T_G = 0.393  # (s)
K_G = 1.05
TTC = -2.5  # (s)

W_D = 0.020
W_V = 0.025
W_A = 5.00

delta_d_max = np.inf
delta_d_min = -np.inf
delta_v_max = 15
delta_v_min = -15
a_f_max = np.inf
a_f_min = -np.inf
control_max = 5
control_min = -5
state_min = np.array([delta_d_min, delta_v_min, a_f_min])
state_max = np.array([delta_d_max, delta_v_max, a_f_max])

def get_des(x): return R*x*(x-V_F_MEAN)+TAU_H*x+D_0
def get_diffdes(x): return TAU_H+R*(2*x-V_F_MEAN)

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
    font = {'family': 'Times New Roman', 'size': '10'}
    font_title = {'family': 'Times New Roman', 'size': '16'}
    if legend is not None:
        plt.legend(legend, loc=legend_loc, ncol=ncol, prop=font)
        # 'lower center'
    plt.xlabel(xlabel, font_title)
    plt.ylabel(ylabel, font_title)
    if xlim is not None:
        plt.xlim(xlim)
    if ylim is not None:
        plt.ylim(ylim)
    plt.tight_layout()
    if title:
        plt.title(title, font_title)
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


def mpcSolver(state, v_p, a_p, opt):
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
    if opt['obs'] == False:
        a_p = 0
    NPC = opt['NPC']
    NP = opt['NP']
    v_f = v_p - state[1]
    state_list = state.tolist()
    zero_list = np.zeros(DYNAMICS_NUM).tolist()

    control = MX.sym('x', ACTIONS_NUM)  # 1*1
    s = MX.sym('s', DYNAMICS_NUM)  # 1*3
    s_next = vertcat(
        s[0] + DT * s[1] - DT * get_diffdes(v_f) * s[2],
        s[1] - DT * s[2] + DT * a_p,
        (1 - DT / T_G) * s[2] + K_G / T_G * DT * control
    )
    get_next = Function('f', [s, control], [s_next])

    loss = W_D*s[0]*s[0]+W_V*s[1]*s[1]+W_A*control*control  # 1*1
    get_loss = Function('L', [s, control], [loss])

    constraint_fixed = -s[0]+(R*(v_p-s[1]-V_F_MEAN)+TAU_H)*s[1]\
        - TAU_H*v_p+D_S0-D_0
    constraint_ttc = -s[0]+(R*(v_p-s[1]-V_F_MEAN)+TAU_H)*s[1]\
        - TAU_H*v_p+ 1/2 * (TTC * s[1] + fabs(TTC * s[1])) +D_S0 - D_0
    get_constraint = Function('H', [s], [constraint_ttc])

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
        if opt['method'] == 'PTW' and time < NPC:  # Pointwise
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
        elif opt['method'] == 'TP' and time == NP - 1:
            g += [get_constraint(s_)]
            g_max += [0]
            g_min += [-np.inf]

        # loss
        L = L+get_loss(s_, u)


    if opt['optimizer'] == 'ipopt':
        solve_options = {'ipopt.print_level': 0,
                         'ipopt.sb': 'yes', 'print_time': 0}
    else:
        solve_options = {}

    solve_instance = {'x': vertcat(*x), 'f': L, 'g': vertcat(*g)}
    solve_function = nlpsol('Func', opt['optimizer'], solve_instance, solve_options)
    # test
    x_initial = [*state_list, 0]*NP
    x_initial += state_list
    control_sequence = solve_function(
        x0=x_initial, ubx=x_max, lbx=x_min, lbg=g_min, ubg=g_max)
    return np.array(control_sequence['x']).reshape(-1), np.array(control_sequence['f'])


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
        v_p = np.load('v_p1.npy')
    elif mode == 5:
        v_p = np.load('v_p3.npy')
    # plt.plot(t_sequence*DT,v_p)
    # plt.title('speed of front car: v_p')
    # plt.xlabel('t/s')
    # plt.ylabel('$v_p/(m\cdot s^{-1})$')
    # plt.show()
    return v_p


def plot_result(des, state, control, distance, vp, save_name=None):

    n = len(state)
    t = np.arange(n) * DT
    data_delta_d = [t, state[:, 0]]
    vp = vp[:n]
    plt.figure(figsize=(15, 8))
    plt.subplot(2, 2, 1)
    myplot(
        data_delta_d,
        figure_num=1,
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
    # data_d = [[t, distance], [t, D_S0 * np.ones(len(t))]]
    data_d = [[t, distance], [t,  0.5 * (np.abs(TTC * state[:, 1]) + TTC * state[:, 1]) + D_S0]]
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
    if save_name:
        plt.savefig(save_name)
    else:
        plt.show()
    plt.close()

def mkLogDir():
    nowtime = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    log_dir = "./Results_dir/" + nowtime
    os.makedirs(log_dir, exist_ok=True)
    return log_dir, nowtime

def save_file(des, state, control, distance, cost, cal_time, v_p, opt):

    log_dir, nowtime = mkLogDir()
    plot_result(des, state, control, distance,
                v_p, save_name=os.path.join(log_dir, 'plot.png'))
    opt['datetime'] = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    print('Mean cost: {:3.3e}'.format(np.mean(cost)))
    print('Mean time: {:3.3e}'.format(np.mean(cal_time)))
    opt['caltime'] = np.mean(cal_time)
    opt['cost'] = np.mean(cost)
    np.savetxt(os.path.join(log_dir, 'des.txt'), des)
    np.savetxt(os.path.join(log_dir, 'state.txt'), state)
    np.savetxt(os.path.join(log_dir, 'control.txt'), control)
    np.savetxt(os.path.join(log_dir, 'distance.txt'), distance)
    np.savetxt(os.path.join(log_dir, 'cost.txt'), cost)
    np.savetxt(os.path.join(log_dir, 'caltime.txt'), cal_time)
    np.savetxt(os.path.join(log_dir, 'v_p.txt'), v_p)
    np.save(os.path.join(log_dir, 'log.npy'), opt)
    f = open(os.path.join(log_dir, 'results.txt'), 'a')
    f.write(str(opt))
    f.close()

    header = [
        'datetime',
        'method',
        'paras',
        'NPC',
        'NP',
        'optimizer',
        'vp_mode',
        'obs',
        'caltime',
        'cost'
        ]
    datas = [opt]
    with open('./runs.csv','a', newline='',encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=header)
        # writer.writeheader()
        writer.writerows(datas)

def plot_comparison_1(dir_list, methods_name):
    font = {'family': 'Times New Roman', 'size': '10'}
    font_title = {'family': 'Times New Roman', 'size': '16'}
    des_d = []
    state_d = []
    control_d = []
    distance_d = []
    cost_d = []
    cal_time_d = []
    v_p_d = []
    constraint_d = []
    data_v = []
    data_d = []
    data_c = []
    data_u = []
    for [i, simulation] in enumerate(dir_list):
        log_dir = "./Results_dir/" + simulation
        des = np.loadtxt(os.path.join(log_dir, 'des.txt'))
        state = np.loadtxt(os.path.join(log_dir, 'state.txt'))
        control = np.loadtxt(os.path.join(log_dir, 'control.txt'))
        distance = np.loadtxt(os.path.join(log_dir, 'distance.txt'))
        cost = 0.025 * (np.multiply(state[:, 0], state[:, 0])) + 0.02 * (np.multiply(state[:, 1], state[:, 1])) + 5 * (
            np.multiply(state[:, 2], state[:, 2]))
        cal_time = np.loadtxt(os.path.join(log_dir, 'caltime.txt'))
        v_p = np.loadtxt(os.path.join(log_dir, 'v_p.txt'))
        constraint = 0.5 * (np.abs(TTC * state[:, 1]) + TTC * state[:, 1]) + D_S0
        n = len(state)
        t = np.arange(n) * DT
        des_d.append([t, des])
        state_d.append(state)
        control_d.append(control)
        distance_d.append(distance)
        cost_d.append(cost)
        cal_time_d.append(cal_time)
        v_p_d.append(v_p)
        constraint_d.append(constraint)
        if i == 0:
            data_v.append([t, v_p])
            data_v.append([t, v_p - state[:, 1]])
            data_d.append([t, 0.5 * (np.abs(TTC * state[:, 1]) + TTC * state[:, 1]) + D_S0])
            data_d.append([t, distance])
            data_c.append([t, 0*t])
            data_c.append([t, distance - constraint])
            data_u.append([t, control])

        else:
            data_v.append([t, v_p - state[:, 1]])
            data_d.append([t, distance])
            data_c.append([t, distance - constraint_d[0]])
            data_u.append([t, control])

    fig = plt.figure(figsize=[9,13],dpi=200)

    plt.subplot(5,1,1)
    methods_name.insert(0, 'Proceeding Vehicle')
    myplot(
        data_v,
        figure_num=i+2,
        legend=methods_name,
        xlabel='(a) time / s',
        ylabel='v / (m/s)',
        title='Velocity',
        xlim=[0, 70],
        ylim=[5, 30],
        legend_loc='upper left',
        ncol=2
    )
    plt.subplot(5,1,2)
    methods_name[0]='Constraints'
    myplot(
        data_d,
        figure_num=2,
        legend=methods_name,
        xlabel='(b) time / s',
        ylabel='d / m',
        xlim=[0, 70],
        title='True Distance',
        legend_loc='upper left',
        ncol=2
    )
    plt.subplot(5,1,3)
    methods_name[0]='Zero'
    myplot(
        data_c,
        figure_num=i + 2,
        legend=methods_name,
        xlabel='(c) time / s',
        ylabel='d / m',
        ylim=[-4, 10],
        xlim=[0, 70],
        legend_loc='lower left',
        title=' Constraint Violation ',
        ncol=4
    )
    # plt.subplot(5, 1, 5)
    # plt.boxplot([cal_time_d[0]],
    #             labels=['GCBF'],
    #             positions=[3],
    #             vert=False,
    #             patch_artist=True,
    #             widths=0.6,
    #             whis=2,
    #             boxprops=dict(facecolor='orange'),
    #             meanprops=dict(color='c')
    #             )
    # plt.boxplot([cal_time_d[1]],
    #             labels=['30-stpes PTW'],
    #             positions=[2],
    #             vert=False,
    #             patch_artist=True,
    #             widths=0.6,
    #             whis=2,
    #             boxprops=dict(facecolor='green'),
    #             meanprops = dict(color='c')
    #             )
    # plt.boxplot([cal_time_d[2]],
    #             labels=['50-steps PTW'],
    #             positions=[1],
    #             vert=False,
    #             patch_artist=True,
    #             widths=0.8,
    #             whis=2,
    #             boxprops=dict(facecolor='red'),
    #             meanprops=dict(color='c')
    #             )
    # plt.title('Single Step Calculation Time')
    # plt.xlabel('(e) time / s', font)
    # plt.grid(True)
    # plt.subplot(5,1,5)
    ax1 = fig.add_subplot(514)
    ax1.plot(t, cost_d[0], label='GCBF')
    # ax1.plot(t, cost_d[1]/30, label='30-steps PTW')
    ax1.plot(t, cost_d[2], label='50-steps PTW')
    # ax1.set_ylim([-10, 80])
    ax1.set_ylabel('Average cost',font_title)
    ax2 = ax1.twinx()
    ax2.plot(t, v_p, 'c--', label='proceeding vehicle speed')
    ax2.set_ylim([10,100])
    ax2.set_ylabel('speed [m/s]',font_title)
    ax1.set_xlim([0,70])
    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    plt.legend(handles1 + handles2, labels1 + labels2, loc='upper right', prop=font)
    # plt.legend('best')
    plt.title('Average single-step cost',font_title)
    ax1.set_xlabel('(d) time/s', font_title)

    plt.subplot(515)
    myplot(
        data_u,
        figure_num=i + 2,
        legend=methods_name[1:4],
        xlabel='(e) time / s',
        ylabel='$a_{fdes}$ / $(m/s^2)$',
        title='Control',
        xlim=[0, 70],
        legend_loc='upper right',
        ncol=3
    )
    plt.show()

def plot_comparison_2(dir_list, methods_name):
    font = {'family': 'Times New Roman', 'size': '12'}
    font_title = {'family': 'Times New Roman', 'size': '18'}
    des_d = []
    state_d = []
    control_d = []
    distance_d = []
    cost_d = []
    cal_time_d = []
    v_p_d = []
    constraint_d = []
    data_v = []
    data_d = []
    data_c = []
    data_t = []
    for [i, simulation] in enumerate(dir_list):
        log_dir = "./Results_dir/" + simulation
        des = np.loadtxt(os.path.join(log_dir, 'des.txt'))
        state = np.loadtxt(os.path.join(log_dir, 'state.txt'))
        control = np.loadtxt(os.path.join(log_dir, 'control.txt'))
        distance = np.loadtxt(os.path.join(log_dir, 'distance.txt'))
        # cost = np.loadtxt(os.path.join(log_dir, 'cost.txt'))
        cost = 0.025* (np.multiply(state[:,0], state[:,0])) + 0.02* (np.multiply(state[:,1], state[:,1])) + 5* (np.multiply(state[:,2], state[:,2]))
        cal_time = np.loadtxt(os.path.join(log_dir, 'caltime.txt'))
        v_p = np.loadtxt(os.path.join(log_dir, 'v_p.txt'))
        constraint = 0.5 * (np.abs(TTC * state[:, 1]) + TTC * state[:, 1]) + D_S0
        n = len(state)
        t = np.arange(n) * DT
        des_d.append([t, des])
        state_d.append(state)
        control_d.append(control)
        distance_d.append(distance)
        cost_d.append(cost)
        cal_time_d.append(cal_time)
        v_p_d.append(v_p)
        constraint_d.append(constraint)
        if i == 0:
            data_v.append([t, v_p])
            data_v.append([t, v_p - state[:, 1]])
            data_d.append([t, 0.5 * (np.abs(TTC * state[:, 1]) + TTC * state[:, 1]) + D_S0])
            data_d.append([t, distance])
            data_c.append([t, 0*t])
            data_c.append([t, distance - constraint])
            data_t.append([t, cal_time])

        else:
            data_v.append([t, v_p - state[:, 1]])
            data_d.append([t, distance])
            data_c.append([t, distance - constraint])
            data_t.append([t, cal_time])

    plt.figure(figsize=[8,4],dpi=400)


    plt.subplot(1, 3, 1)
    plt.boxplot([cal_time_d[0]],
                labels=['GCBF'],
                positions=[1],
                vert=True,
                patch_artist=True,
                widths=0.6,
                whis=2,
                )
    plt.boxplot([ cal_time_d[3]],
                labels=['PTW'],
                positions=[2],
                vert=True,
                patch_artist=True,
                widths=0.6,
                whis=2,
                boxprops=dict(facecolor='c')
                )
    plt.title('Ipopt')
    plt.ylabel('time / s', font)
    plt.grid(True)
    plt.subplot(1, 3, 2)
    plt.boxplot([cal_time_d[1]],
                labels=['GCBF'],
                positions=[1],
                vert=True,
                patch_artist=True,
                widths=0.6,
                whis=2,
                )
    plt.boxplot([cal_time_d[4]],
                labels=['PTW'],
                positions=[2],
                vert=True,
                patch_artist=True,
                widths=0.6,
                whis=2,
                boxprops=dict(facecolor='c')
                )
    plt.title('SQPMethod')
    plt.ylabel('time / s', font)
    plt.grid(True)
    plt.subplot(1, 3, 3)
    plt.boxplot([cal_time_d[2]],
                labels=['GCBF'],
                positions=[1],
                vert=True,
                patch_artist=True,
                widths=0.6,
                whis=2,
                )
    plt.boxplot([cal_time_d[5]],
                labels=['PTW'],
                positions=[2],
                vert=True,
                patch_artist=True,
                widths=0.6,
                whis=2,
                boxprops=dict(facecolor='c')
                )
    plt.title('Bonmin')
    plt.ylabel('time / s', font)
    plt.grid(True)
    # plt.savefig('plot.png')
    plt.show()

if __name__ == '__main__':
    dir_list = ['2020-11-09-13-11-53','2020-11-09-13-18-56','2020-11-09-13-08-02']
    # dir_list = ['2020-11-09-12-42-30', '2020-11-08-22-31-08', '2020-11-09-12-40-52']
    # change vp_mode = 5
    dir_list = ['2020-11-14-14-13-44', '2020-11-14-14-10-44', '2020-11-14-14-11-11']
    dir_list_2 = ['2020-11-07-19-15-26','2020-11-07-19-23-30','2020-11-07-19-24-04','2020-11-07-19-24-45','2020-11-07-19-34-51', '2020-11-07-19-35-36']
    methods_name = ['GCBF','10-steps PTW','50-steps PTW']
    methods_name_2 = ['CBF-ipopt','CBF-SQPMethod','CBF-Bonmin','PTW-ipopt','PTW-SQPMethod','PTW-Bonmin']
    plot_comparison_1(dir_list,methods_name)
    # plot_comparison_2(dir_list_2, methods_name_2)