# (c) https://github.com/MontiCore/monticore  
import matplotlib.pyplot as plt
import numpy as np

class AccuracyFig:
    def __init__(self, ax):
        self.ax = ax

#        self.acc_real, = self.ax.plot([],[], c="green", marker=2, ms=30, markeredgewidth=3)
#        self.acc_pred, = self.ax.plot([],[], c="red", marker=3, ms=30, markeredgewidth=3)

#        self.acc_real, = self.ax.plot([],[], c="green", marker="|", ms=60, markeredgewidth=2)
#        self.acc_pred, = self.ax.plot([],[], c="red", marker="|", ms=60, markeredgewidth=1)
#        self.TEST, = self.ax.plot([0],[0.4], c="red", marker=".", ms=0.1, markeredgewidth=1)

        self.acc_real_width = 1
        self.acc_pred_width = 0.5
        self.height = 0.5
        self.offset = 0.2

        self.acc_real = self.ax.add_patch(plt.Rectangle((0,0), 0, 0, color="green"))
        self.acc_pred = self.ax.add_patch(plt.Rectangle((0,0), 0, 0, color="red"))

        self.error_val = self.ax.text(0,0,"none", color="red")
        self.error = self.ax.add_patch(plt.Rectangle((0,0), 0, 0, color="#ff918f", zorder=0))

    def update(self, val_real, val_pred):
        val_real = np.degrees([val_real])
        val_pred = np.degrees([val_pred])

#        self.acc_real.set_data(val_real,0.40)
#        self.acc_pred.set_data(val_pred,0.40)

        self.acc_real.set_bounds(val_real,self.offset,self.acc_real_width,self.height)
        self.acc_pred.set_bounds(val_pred,self.offset,self.acc_real_width,self.height)

        x = min(val_real, val_pred)
        width = max(val_real, val_pred) - x
        self.error_val.set_text(round(width[0], 1))
        self.error_val.set_position((x+width/2.-10,self.height+self.offset+0.05))
        self.error.set_bounds(x,self.offset,width,self.height)

