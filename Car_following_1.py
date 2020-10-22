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
T_G = 0.393
K_G = 1.05
TTC=-2.5 # (s)
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
state_min=np.array([delta_d_min,delta_v_min,a_f_min])
state_max=np.array([delta_d_max,delta_v_max,a_f_max])

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
    '''
    # des = get_des(v_p-state[1])  # expected distance

    control=MX.sym('x',ACTIONS_NUM)#1*1
    s=MX.sym('s',DYNAMICS_NUM)#1*3
    zero_NP=np.zeros(NP).tolist()
    state_np=np.array(state)
    loss=0.5*(s[1]**2+control**2)#1*1
    s_next=vertcat(
            s[0]+DT*s[1]-DT*get_diffdes(v_p)*s[2],
            s[1]-DT*s[2],
            (1-DT/T_G)*s[2]+K_G/T_G*DT*control
        )
    get_next=Function('f',[s,control],[s_next])
    get_loss=Function('L',[s,control],[loss])

    x=[None]*2*NP
    x_min=[None]*2*NP
    x_max=[None]*2*NP
    for time in range(NP):
        #control
        x[2*time]=MX.sym('u'+str(i))
        x_min[2*time] = control_min
        x_max[2*time] = control_max
        #state
        x[2*time+1] = get_next(s,x[2*time])
        x_min
    g = [s]
    g_min = [*state]
    g_max = [*state]
    J = 0

    for time in range(NP):
        s=get_next(s,u[time])#get the next state
        if time<=1: # first and second time
            g.append(s) # attach the barrier constraint
            g_min+=[state_np*(1-cbf_lambda)+cbf_lambda*state_min]
            g_max+=[state_np*(1-cbf_lambda)+cbf_lambda*state_max]

        J+=get_loss(s,u[time])

    solve_instance={'x':vertcat(*u),'f':J,'g':vertcat(*g)}
    solve_function=nlpsol('Func','ipopt',solve_instance)#s are free?
    #testing
    control_sequence=solve_function(x0=zero_NP,ubx=u_max,lbx=u_min,lbg=g_min,ubg=g_max)
    return np.array(control_sequence['x'])
    




def generate_vp(tmax):  # generate v_p sequence
    half = math.floor(tmax/2)
    t_sequence = np.arange(tmax)
    v_p1 = 10+5*np.sin(t_sequence[:half]*DT*np.pi/10)
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
    cbf_lambda = 0.2
    max_time = 600
    i = 0
    main_loop(max_time)
