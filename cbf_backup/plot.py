import numpy as np
import matplotlib.pyplot as plt

def plot_feasibility(data, k, para):
    position = data[:,[0,1]]
    feasibility = data[:,2]
    print(para[1], 600-np.sum(feasibility))
    plt.figure(k)
    for i in range(len(data)):
        if feasibility[i] == 0:
            plt.scatter(position[i, 0], position[i, 1], marker='s', color='limegreen')
        else:
            plt.scatter(position[i, 0], position[i, 1], marker='x', color='gold')
    plt.xlabel('d (m)')
    plt.ylabel('v (m/s)')
    if para[1] == 1:
        plt.title("Pointwise constraint, Np=" + str(para[0]))
    else:
        plt.title("Barrier constraint, Np="+ str(para[0]) + ", $\lambda=$" + str(para[1]))
    plt.show()
if __name__ == '__main__':

    plot_feasibility(np.loadtxt('history1.txt'),0,[6,1])
    plot_feasibility(np.loadtxt('history0.7.txt'), 1, [6, 0.7])
    plot_feasibility(np.loadtxt('history0.4.txt'), 2, [6, 0.4])
    plot_feasibility(np.loadtxt('history0.1.txt'), 3, [6, 0.1])
    plot_feasibility(np.loadtxt('history0.01.txt'), 4, [6, 0.01])
    plot_feasibility(np.loadtxt('history0.01.txt'), 5, [30, 1])
