import numpy as np
import matplotlib.pyplot as plt
def insert_zero(test):
    i = 0
    # test = [1,2,3,4]
    while True:
        if test[i] >= 15.9:
            test = np.insert(test, i, 16.0*np.ones([10,]), axis=0)
            i += 10
        elif test[i] <= 10.1:
            test = np.insert(test, i, 10.0*np.ones([10,]), axis=0)
            i += 10
        i += 1
        if i >= len(test):
            break
    #@print(test)

    return test
def insert_zero2(test):
    i = 0
    # test = [1,2,3,4]
    while True:
        if test[i] >= 26.4:
            test = np.insert(test, i, 26.5*np.ones([10,]), axis=0)
            i += 10
        elif test[i] <= 23.6:
            test = np.insert(test, i, 23.5*np.ones([10,]), axis=0)
            i += 10
        i += 1
        if i >= len(test):
            break
    print(test)

    return test

def flattern(input):
    output = input
    for i in range(len(input)-7):
        output[3+i] = np.mean(input[i:i+7])
    return output

def generate_sin(mean, volumn, init_phase, period, times):
    x = np.arange(period * times) / (4 * period) * 2 * np.pi
    y = mean + volumn * np.sin(x + init_phase)
    return y

def modify_vp():
    vp = np.load('v_p1.npy')
    v_p1 = vp[0: 283]
    v_p2 = vp[283:400]
    v_p3 = vp[400:486]
    v_p4 = vp[486:-1]


    # v_p1_m = flattern(insert_zero(v_p1))
    # v_p3_m = flattern(insert_zero2(v_p3))
    v_p1_m = np.concatenate((generate_sin(13,3,3/2*np.pi,10,12),generate_sin(13,3,3/2*np.pi,6,12),generate_sin(13,3,3/2*np.pi,8,12)),axis=0)
    v_p3_m = generate_sin(25,1.5,0,5,13)
    v_p_m = np.concatenate((v_p1_m,v_p2,v_p3_m,v_p4),axis=0)

    plt.plot(range(len(v_p_m)), v_p_m)
    plt.show()
    v_p_m = np.append(v_p_m,15 * np.ones([17,]))
    print(len(v_p_m))
    np.save('v_p3.npy',v_p_m)

if __name__ == '__main__':
    import os
    os.chdir('D:\\CppAndPython\\GitTest\\new_20201115')
    modify_vp()
