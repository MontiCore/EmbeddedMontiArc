import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import re
import os
import time

LOGPATH = "../../logs/"
SAVEPATH = "../../"
#NAMES = ["train-senetSmall-separable", "SE-Net", "train-resnet-shuffle"] 
#NAMES = ["ResNet-noShuffle"]
#NAMES = ["train-senetSmall-separable"] 
#NAMES = ["AlexNet-Small", "train-resnet-shuffle","SE-Net", "train-senetSmall-separable"]
NAMES = ["AlexNetSmall", "ResNetSmall", "SE-NetSmall", "SE-NetSmall*"]

BATCH_START = 0
BATCH_END = int(100000/64)

VALIDATION_SIZE = 100

X_AXIS_BATCH = False # use itterations instead if false
Y_SCALE= "log"

SAVE = False



def get_train_error(filename):
    rows = open(LOGPATH + filename + ".log").read().strip()
    train_error = []
    validation_error = []
    train_iterations = []
    validation_iterations = []
    # grab the set of training epochs
    epochs = set(re.findall(r'Epoch\[(\d+)\]', rows))
    epochs = sorted([int(e) for e in epochs])
    for e in epochs:
        step = re.findall(r'(Epoch\[' + str(e) + '\]) (.*)', rows)
        for row in step:
            cur_str = row[1]
            train_error_str = re.findall(r'(Loss:)\s(\d*\.\d*)' , cur_str)
            if len(train_error_str) > 0:
                train_error.append(float(train_error_str[0][1]))
    return train_error



def plot_error(train_errors, names):
    if SAVE:
        matplotlib.use("pgf")
        matplotlib.rcParams.update({
            "pgf.texsystem": "pdflatex",
            'font.family': 'serif',
            'text.usetex': True,
            'pgf.rcfonts': False,
        })
     # plt.style.use("ggplot")
    fig = plt.figure()
    ax = fig.add_subplot(2, 1, 1)
    ax.set_yscale(Y_SCALE)
    # to do title
    if X_AXIS_BATCH:
        plt.xlabel("Batches")
    else:
        plt.xlabel(r"$10^3$" + " Iterations")

    plt.ylabel("Error")
    for train_error, name in zip(train_errors, names):
        if X_AXIS_BATCH:
            plt.plot(train_error[BATCH_START: BATCH_END], label="train error")
        else:
            x = np.arange(min(len(train_error) - BATCH_START, BATCH_END - BATCH_START)) * 64 * 1e-3
            plt.plot(x, train_error[BATCH_START: BATCH_END], label="train error")
        

    if len(names) >= 2:
        plt.legend(names)
    if SAVE:
        plt.savefig(SAVEPATH + "todo" + "_error_plot_")
    else:
        plt.show()


def getFilename(name):
    files = os.listdir("../logs")
    tmp = []
    for file_name in files:
        if len(re.findall(r'' + name, file_name)) > 0:
            tmp.append(file_name)
    


if __name__ == "__main__":
    train_errors = []

    for name in NAMES:
        err = get_train_error(name)
        train_errors.append(err)
        print(name + " trained for %i Batches (%i Iterations)" % (len(err), len(err) * 64))
        print("train error of %.2e" %(sum(err[-VALIDATION_SIZE: ]) / VALIDATION_SIZE))
        print()
    plot_error(train_errors, NAMES)

