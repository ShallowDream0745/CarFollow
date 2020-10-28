import matplotlib.pyplot as plt

def myplot(data,
           figure_num=1,
           mode="xy",
           fname=None,
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

    _, ax = plt.subplots()
    if figure_num == 1:
        data = [data]

    if color_list is not None:
        for (i, d) in enumerate(data):
            if mode == "xy":
                plt.plot(d[0], d[1], color=color_list[i])
            if mode == "y":
                plt.plot(d, color=color_list[i])
            if mode == "scatter":
                plt.scatter(d[0], d[1], color=color_list[i], marker=".", s =5.,)
    else:
        for (i, d) in enumerate(data):
            if mode == "xy":
                plt.plot(d[0], d[1])
            if mode == "y":
                plt.plot(d)
            if mode == "scatter":
                plt.scatter(d[0], d[1], marker=".", s =5.,)

    plt.tick_params(labelsize=18)
    labels = ax.get_xticklabels() + ax.get_yticklabels()
    [label.set_fontname('Calibri') for label in labels]
    font = {'family': 'Calibri', 'size': '18'}
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

    if fname is None:
        plt.show()
    else:
        plt.savefig(fname)