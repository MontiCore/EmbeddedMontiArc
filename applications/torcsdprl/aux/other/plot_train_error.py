import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import re

TITLE = "AlexNet"
LOGFILE = "../../train.log"
SAVEPATH = "../../"
SAVE = False
IGNORE_FIRST_BATCHES = 0

rows = open(LOGFILE).read().strip()
train_error = list()
validation_error = list()
train_iterations = list()
validation_iterations = list()
speeds = list() # to do

if SAVE:
    matplotlib.use("pgf")
    matplotlib.rcParams.update({
        "pgf.texsystem": "pdflatex",
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
    })

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
    #train_error.append(1 - float(re.findall(r'(Train:)\s(\d\.\d*)' , cur_str)[0][1]))
    #validation_error.append(1 - float(re.findall(r'(Test:)\s(\d\.\d*)' , cur_str)[0][1]))


# plt.style.use("ggplot")
fig = plt.figure()
ax = fig.add_subplot(2, 1, 1)
ax.set_yscale("linear")
plt.plot(train_error[IGNORE_FIRST_BATCHES:], label="train error")
plt.xlabel("Batch")
plt.ylabel("Error")
plt.title(TITLE)
if SAVE:
    plt.savefig(SAVEPATH + TITLE + "_error_plot_")
else:
    plt.show()