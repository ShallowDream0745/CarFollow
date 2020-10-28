from Convergence import Convergence
import numpy as np
import matplotlib.pyplot as plt

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
                plt.scatter(d,u,marker='s',color='limegreen')
                stop1 += 1
            else:
                plt.scatter(d,u,marker='x',color='gold')
            history.append([d,u,crash])
    plt.xlabel('d (m)')
    plt.ylabel('u (m/s)')
    plt.title("Barrier constraints, Np=6, $\lambda=$"+str(cbf_para))
    plt.show()

    print(cbf_para, stop1)
    name = 'history' + str(cbf_para) + '.txt'
    np.savetxt(name, np.array(history))

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