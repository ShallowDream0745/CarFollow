"""
    <Reinforcement Learning and Control>(Year 2020)
    by Shengbo Eben Li
        @ Intelligent Driving Lab, Tsinghua University

    OCP example for lane keeping problem in a circle road

    [Method]
    Open loop solution

"""
from casadi import *
from Config import DynamicsConfig
import math


class Solver(DynamicsConfig):
    """
    NLP solver for nonlinear model predictive control with Casadi.
    """
    def __init__(self):
        self.U_LOWER = - 10
        self.U_UPPER = 0
        self.alpha1 = self.beta * (self.x_init[1] / self.x_init[0] + self.alpha)
        self.alpha2 = (self.x_init[1] / self.x_init[0] + self.alpha + self.beta)
        self._sol_dic = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        self.X_init = [0.0, 0.0, 0.1, 0.0, 0.0]
        self.zero = [0., 0.]
        self.x_last = 0
        super(Solver, self).__init__()

    def mpcSolver(self, x_init, predict_steps, CBF, cbf_para):
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
        self.f_l = vertcat(
            x[0] + self.Ts * (- x[1]),
            x[1] + self.Ts * ( u[0] )
        )



        # Create solver instance

        self.F = Function("F", [x, u], [self.f_l])

        # Create empty NLP
        w = []
        lbw = []
        ubw = []
        lbg = []
        ubg = []
        G = []
        J = 0

        # Initial conditions
        Xk = MX.sym('X0', self.DYNAMICS_DIM)
        w += [Xk]
        lbw += x_init
        ubw += x_init

        for k in range(1, predict_steps + 1):
            # Local control
            Uname = 'U' + str(k - 1)
            Uk = MX.sym(Uname, self.ACTION_DIM)
            w += [Uk]
            lbw += [self.U_LOWER]
            if CBF == 1:
                self.u_upper = cbf_para[0] * x_init[0] - cbf_para[1] * x_init[1]
                if k == 1:
                    if self.u_upper >= self.U_UPPER:
                        ubw += [self.U_UPPER]
                    elif self.u_upper <= self.U_LOWER:
                        ubw += [self.U_LOWER]
                    else:
                        ubw += [self.u_upper]
                else:
                    ubw += [self.U_UPPER]
            else:
                ubw += [self.U_UPPER]
            Fk = self.F(Xk, Uk)
            Xname = 'X' + str(k)
            Xk = MX.sym(Xname, self.DYNAMICS_DIM)


            # Dynamic Constriants
            G += [Fk - Xk]
            lbg += self.zero
            ubg += self.zero
            w += [Xk]
            if k == 2:
                constraint = (1-lambda_cbf)* (1-lambda_cbf) * x_init[0]
                lbw += [constraint , 0]
                # print("constraint: ",constraint)
            else:
                lbw += [-20,0]
            ubw += [inf, 50]


            # Cost function
            # F_cost = Function('F_cost', [x, u], [1 * (u[0]) ** 2])
            F_cost = Function('F_cost', [x, u], [1 * (x[1] - x_init[1]) ** 2])
            J += F_cost(w[k * 2], w[k * 2 - 1])


        # Create NLP solver
        nlp = dict(f=J, g=vertcat(*G), x=vertcat(*w))
        S = nlpsol('S', 'ipopt', nlp, self._sol_dic)

        # Solve NLP
        r = S(lbx=lbw, ubx=ubw, x0=0, lbg=lbg, ubg=ubg)
        # print(r['x'])
        state_all = np.array(r['x'])
        state = np.zeros([predict_steps, self.DYNAMICS_DIM])
        control = np.zeros([predict_steps, self.ACTION_DIM])
        nt = self.DYNAMICS_DIM  + self.ACTION_DIM  # total variable per step

        # save trajectories
        for i in range(predict_steps):
            state[i] = state_all[nt * i: nt * i + nt - 1].reshape(-1)
            control[i] = state_all[nt * i + nt - 1]
        return state, control
