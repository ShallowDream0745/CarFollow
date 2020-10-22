"""
    <Reinforcement Learning and Control>(Year 2020)
    by Shengbo Eben Li
        @ Intelligent Driving Lab, Tsinghua University

    OCP example for lane keeping problem in a circle road

    [Method]
    Open loop solution

"""
from casadi import *
import math
import numpy as np
import matplotlib.pyplot as plt


class DynamicsConfig(object):
    DYNAMICS_DIM = 3
    ACTION_DIM = 1
    delta_t = 0.05  # control signal period
    Np = 6  # predict horizon
    lambda_cbf = 1 / 23  # lambda in control barrier function
    x_init = [25, 20]
    alpha = 0.1
    beta = 0.1
    CBF = 1  # whether using CBF


def plot_feasibility(data, k, para):
    position = data[:, [0, 1]]
    feasibility = data[:, 2]
    print(para[1], 600 - np.sum(feasibility))
    plt.figure(k)
    for i in range(len(data)):
        if feasibility[i] == 0:
            plt.scatter(position[i, 0],
                        position[i, 1],
                        marker='s',
                        color='limegreen')
        else:
            plt.scatter(position[i, 0],
                        position[i, 1],
                        marker='x',
                        color='gold')
    plt.xlabel('d (m)')
    plt.ylabel('v (m/s)')
    if para[1] == 1:
        plt.title("Pointwise constraint, Np=" + str(para[0]))
    else:
        plt.title("Barrier constraint, Np=" + str(para[0]) + ", $\lambda=$" +
                  str(para[1]))
    plt.show()


class Convergence(DynamicsConfig):
    def examine(self, x, CBF, cbf_para):
        crash = 0
        x_traj = [x]
        u_traj = []
        solver = Solver()
        for i in range(1000):
            state, control = solver.mpcSolver(x, self.Np, CBF, cbf_para)
            state = state.tolist()
            control = control.tolist()
            u = control[0]
            x_next = [x[0] - self.delta_t * x[1], x[1] + self.delta_t * u[0]]
            x = x_next
            x_traj.append(x)
            u_traj.append(u)
            # print(x[1])
            if x[1] <= 0.01:
                # print('Stop!')
                break
            if x[0] <= 0:
                # print('Crash!')
                crash = 1
                break
        return crash


class Solver(DynamicsConfig):
    """
    NLP solver for nonlinear model predictive control with Casadi.
    """

    def __init__(self):
        self.tau_h = 1.68
        self.r = 0.054
        self.v_f_mean = 15  # m/s
        self.T_G = 0.393
        self.K_G = 1.05

        self.U_LOWER = -10
        self.U_UPPER = 0

        # self.alpha1 = self.beta * (self.x_init[1] / self.x_init[0] +
        #                            self.alpha)
        # self.alpha2 = (self.x_init[1] / self.x_init[0] + self.alpha +
        #                self.beta)

        self._sol_dic = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }

        self.X_init = [0.0, 0.0, 0.1, 0.0, 0.0]
        self.zero = [0., 0.]
        self.x_last = 0
        super(Solver, self).__init__()

    def mpcSolver(self, x_init, predict_steps, CBF, cbf_para,):
        """
        Solver of nonlinear MPC

        Parameters
        ----------
        x_init: list
            input state for MPC.
        predict_steps: int
            steps of predict horizon.
        CBF: bool
            whether using continuous barrier constraints
        cbf_para:
            if CBF == 1
                tuple: [alpha, beta]
            if CBF == 0
                float: lambda

        Returns
        ----------
        state: np.array     shape: [predict_steps+1, state_dimension]
            state trajectory of MPC in the whole predict horizon.
        control: np.array   shape: [predict_steps, control_dimension]
            control signal of MPC in the whole predict horizon.
        """

        if CBF == 0:
            lambda_cbf = cbf_para
        if CBF == 1:
            lambda_cbf = 1

        x = SX.sym('x', self.DYNAMICS_DIM)
        u = SX.sym('u', self.ACTION_DIM)

        # lateral ACC model
        self.f_l = vertcat(x[0] + self.delta_t * (x[1]+(-tau_h-r*(2*v_p-self.v_f_mean))*x[2]),
                           x[1] - self.delta_t * (x[2]),
                           (1 - self.delta_t / self.T_G)*x[2]+self.K_G/self.T_G*self.delta_t*u)

        # Create solver instance
        self.F = Function("F", [x, u], [self.f_l])

        # Create empty NLP
        w = []
        u_lower_bound = []
        u_upper_bound = []
        g_lower_bound = []
        g_upper_bound = []
        G = []
        J = 0

        # Initial conditions
        Xk = MX.sym('X0', self.DYNAMICS_DIM)
        w += [Xk]
        u_lower_bound += x_init
        u_upper_bound += x_init

        for k in range(1, predict_steps + 1):
            # Local control
            Uname = 'U' + str(k - 1)
            Uk = MX.sym(Uname, self.ACTION_DIM)
            w += [Uk]
            u_lower_bound += [self.U_LOWER]
            if CBF == 1:
                self.u_upper = cbf_para[0] * \
                    x_init[0] - cbf_para[1] * x_init[1]
                if k == 1:
                    if self.u_upper >= self.U_UPPER:
                        u_upper_bound += [self.U_UPPER]
                    elif self.u_upper <= self.U_LOWER:
                        u_upper_bound += [self.U_LOWER]
                    else:
                        u_upper_bound += [self.u_upper]
                else:
                    u_upper_bound += [self.U_UPPER]
            else:
                u_upper_bound += [self.U_UPPER]
            Fk = self.F(Xk, Uk)
            Xname = 'X' + str(k)
            Xk = MX.sym(Xname, self.DYNAMICS_DIM)

            # Dynamic Constriants
            G += [Fk - Xk]
            g_lower_bound += self.zero
            g_upper_bound += self.zero
            w += [Xk]
            if k == 2:
                constraint = (1 - lambda_cbf) * (1 - lambda_cbf) * x_init[0]
                u_lower_bound += [constraint, 0]
                # print("constraint: ",constraint)
            else:
                u_lower_bound += [-20, 0]
            u_upper_bound += [inf, 50]

            # Cost function
            F_cost = Function('F_cost', [x, u], [1 * (x[1] - x_init[1])**2])
            J += F_cost(w[k * 2], w[k * 2 - 1])

        # Create NLP solver
        nlp = dict(f=J, g=vertcat(*G), x=vertcat(*w))
        S = nlpsol('S', 'ipopt', nlp, self._sol_dic)

        # Solve NLP
        r = S(lbx=u_lower_bound, ubx=u_upper_bound, x0=0, lbg=g_lower_bound, ubg=g_upper_bound)
        # print(r['x'])
        state_all = np.array(r['x'])
        state = np.zeros([predict_steps, self.DYNAMICS_DIM])
        control = np.zeros([predict_steps, self.ACTION_DIM])
        nt = self.DYNAMICS_DIM + self.ACTION_DIM  # total variable per step

        # save trajectories
        for i in range(predict_steps):
            state[i] = state_all[nt * i:nt * i + nt - 1].reshape(-1)
            control[i] = state_all[nt * i + nt - 1]
        return state, control


def single_run(cbf_para, CBF, fig_num):
    d_range = 2 * 10
    u_range = 2 * 15

    plt.figure(fig_num)
    convergence = Convergence()
    stop1 = 0
    history = []
    for i in range(d_range):
        for j in range(u_range):
            d = 0.5 * i
            u = 0.5 * j
            crash = convergence.examine([d, u], CBF, cbf_para)
            if crash == 0:
                plt.scatter(d, u, marker='s', color='limegreen')
                stop1 += 1
            else:
                plt.scatter(d, u, marker='x', color='gold')
            history.append([d, u, crash])
    plt.xlabel('d (m)')
    plt.ylabel('u (m/s)')
    plt.title("Barrier constraints, Np=6, $\lambda=$" + str(cbf_para))
    plt.show()

    print(cbf_para, stop1)
    # name = 'history' + str(cbf_para) + '.txt'
    # np.savetxt(name, np.array(history))


if __name__ == '__main__':
    CBF = 0
    cbf_para = 1
    i = 0
    single_run(cbf_para, CBF, i)
    cbf_para = 0.7
    i += 1
    single_run(cbf_para, CBF, i)
    cbf_para = 0.4
    i += 1
    single_run(cbf_para, CBF, i)
    cbf_para = 0.1
    i += 1
    single_run(cbf_para, CBF, i)
    cbf_para = 0.01
    i += 1
    single_run(cbf_para, CBF, i)
