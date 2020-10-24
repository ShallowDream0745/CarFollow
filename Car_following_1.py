from casadi import *
import math
import numpy as np
import matplotlib.pyplot as plt

DYNAMICS_NUM = 3
ACTIONS_NUM = 1

DT = 0.1  # time intervals
NP = 6  # prediction steps

TAU_H = 1.68  # (s^-1)
R = 0.054  # (s^2 m^-1)
V_F_MEAN = 15  # (m/s)
D_0 = 2.9  # (m)
D_S0 = 5  # (m)
T_G = 0.393  # (s)
K_G = 1.05
TTC = -2.5  # (s)

W_D = 0.8
W_V = 1
W_A = 5


def get_des(x): return R*x*(x-V_F_MEAN)+TAU_H*x+D_0
def get_diffdes(x): return TAU_H+R*(2*x-V_F_MEAN)


delta_d_max = np.inf
delta_d_min = 0
delta_v_max = 10
delta_v_min = -10
a_f_max = np.inf
a_f_min = -np.inf
control_max = 1.5
control_min = -1.5
state_min = np.array([delta_d_min, delta_v_min, a_f_min])
state_max = np.array([delta_d_max, delta_v_max, a_f_max])


def is_violating(control, state, v_p):

    if (state < state_max or
        state > state_min or
        control > control_max or
            control < control_min):
        return True
    return False


def mpcSolver(state, v_p):
    '''
    state:   (d-des,v_p-v_f,a_f)
    v_p: speed of the front car
    des: expected distance between the car
    a_f: acceleration
    u:   control variable
    g:   constraint variable

    RETURN:
    control: (int)
    '''

    state_list =list(state)
    zero_list = np.zeros(DYNAMICS_NUM).tolist()

    control = MX.sym('x', ACTIONS_NUM)  # 1*1
    s = MX.sym('s', DYNAMICS_NUM)  # 1*3
    s_next = vertcat(
            s[0]+DT*s[1]-DT*get_diffdes(v_p)*s[2],
            s[1]-DT*s[2],
            (1-DT/T_G)*s[2]+K_G/T_G*DT*control
        )
    get_next = Function('f', [s, control], [s_next])

    loss = W_D*s[0]*s[0]+W_V*s[1]*s[1]+W_A*control*control  # 1*1
    get_loss = Function('L', [s, control], [loss])

    constraint = -s[0]+(R*(v_p-s[1]-V_F_MEAN)+TAU_H)*s[1]\
                - TAU_H*v_p+D_S0-D_0
    get_constraint=Function('H',[s],[constraint])
    
    x = [None]*(2*NP+1)  # independent variable
    x_min = [None]*(2*NP+1)
    x_max = [None]*(2*NP+1)

    x[0] = MX.sym('s0', DYNAMICS_NUM)  # 1*3
    x_min[0] = state_list
    x_max[0] = state_list

    g = [None]*2*NP
    g_min = [None]*2*NP
    g_max = [None]*2*NP

    L = 0

    for time in range(NP):  # time:[0,NP-1]
        # control
        u = MX.sym('u'+str(time))#1*1
        x[2*time+1] = u
        x_min[2*time+1] = control_min
        x_max[2*time+1] = control_max
        # state
        s_ = MX.sym('s'+str(time+1),DYNAMICS_NUM)#1*3
        x[2*time+2] = s_
        x_min[2*time+2] = state_min.tolist()
        x_max[2*time+2] = state_max.tolist()
        # constraint
        g[2*time] = s_-get_next(x[2*time], u)#1*3
        g_max[2*time] = zero_list
        g_min[2*time] = zero_list
        if not IS_USING_CBF:#Pointwise
            g[2*time+1] = get_constraint(s_)
            g_max[2*time+1]=0
            g_min[2*time+1]=-np.inf
        # loss
        L=L+get_loss(s_,u)
    
    solve_instance = {'x': vertcat(*x), 'f': L, 'g': vertcat(*g)}
    solve_function = nlpsol('Func', 'ipopt', solve_instance)  
    #test
    x_initial=[state_list,0]*NP
    x_initial.extend([state_list])
    control_sequence= solve_function(x0=x_initial, ubx=x_max, lbx=x_min, lbg=g_min, ubg=g_max)
    return np.array(control_sequence['x'])





def generate_vp(tmax):  # generate v_p sequence
    half = math.floor(tmax/2)
    t_sequence = np.arange(tmax)
    v_p1= 10+5*np.sin(t_sequence[: half]*DT*np.pi/10)
    v_p2 = np.ones(tmax-half)*v_p1[-1]
    v_p = np.r_[v_p1, v_p2]
    plt.plot(t_sequence, v_p)
    plt.show()
    return v_p


def main_loop(tmax):
    v_p = generate_vp(tmax)
    state = (0, v_p[0], 0)  # suppose v_f[0]=v_p[0]
    for t in range(tmax):
        # get the control value and the next state
        control = mpcSolver(state, v_p[t])[0]
        state_ = np.array([
            [x[0]+DT*x[1]-DT*get_diffdes(v_p)*x[2]],
            [x[1]-DT*x[2]],
            [(1-DT/T_G)*x[2]+K_G/TG*DT*control]
        ])
        if is_violating(control, x_, v_p):
            return False
    return True


if __name__ == '__main__':
    IS_USING_CBF = False
    #cbf_lambda = 0.2
    max_time = 600
    i = 0
    main_loop(max_time)
