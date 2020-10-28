from Solver import Solver
import numpy as np
import matplotlib.pyplot as plt
from Config import DynamicsConfig

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
            x_next = [x[0] - self.Ts * x[1], x[1] + self.Ts * u[0]]
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
        # plt.figure()
        # x_traj = np.array(x_traj)
        # plt.plot(range(len(x_traj)), -1 * x_traj[:, 0], label='d')
        # plt.plot(range(len(x_traj)), x_traj[:, 1], label='u')
        # plt.legend(loc='upper left')
        # plt.figure()
        # plt.plot(range(len(x_traj) - 1), u_traj)
        # plt.show()



if __name__ == '__main__':

    config = DynamicsConfig()
    x_init = [23,20]
    config.Np = 6
    # config.lambda_cbf = 0.1
    x = x_init
    x_traj = [x_init]
    u_traj = []
    CBF = 0
    cbf_para = [1, 1]
    solver = Solver()
    for i in range(1000):
        state, control = solver.mpcSolver(x, config.Np, CBF, cbf_para)
        state = state.tolist()
        control = control.tolist()
        s_traj = []
        s = x
        # for i in range(config.Np):
        #
        #     s_next = [s[0] - config.Ts * s[1], s[1] + config.Ts * control[i][0]]
        #     s_traj.append(s_next)
        #     s = s_next
        # print(s_traj)
        # print(state[1:])
        u = control[0]
        # u= [-10]
        x_next=[x[0]-config.Ts * x[1], x[1]+config.Ts * u[0]]
        # x_next = state[1]
        x = x_next
        x_traj.append(x)
        u_traj.append(u)
        print(x)
        if x[1] <= 0.01:
            print('Stop!')
            break
        if x[0] <= 0:
            print('Crash!')
            break
    plt.figure()
    x_traj = np.array(x_traj)
    plt.plot(range(len(x_traj)), -1 * x_traj[:, 0], label='d')
    plt.plot(range(len(x_traj)), x_traj[:, 1],label='u')
    plt.legend(loc='upper left')
    plt.figure()
    plt.plot(range(len(x_traj)-1), u_traj)
    plt.show()
