# (c) https://github.com/MontiCore/monticore  
import matplotlib.pyplot as plt
import numpy as np

class WheelFig:
    def __init__(self, ax, position, radius, color='blue'):
        self.ax = ax
        self.position = position
        self.radius = radius
        self.color = color
        self.theta = 0
        self.circle = self.ax.add_patch(plt.Circle(self.position, self.radius, fill=False, lw=8))
        self.circle_inner = self.ax.add_patch(plt.Circle(self.position, 0.05, fill=True, lw=8, color="black"))
        self.line_upper = self.ax.add_line(plt.Line2D([0,0],[0,0],lw=6, c="black"))
        self.line_left = self.ax.add_line(plt.Line2D([0,0],[0,0],lw=6, c="black"))
        self.line_right = self.ax.add_line(plt.Line2D([0,0],[0,0],lw=6, c="black"))
        self.circle_outer = self.ax.add_patch(plt.Circle(self.position, 0.05, fill=True, lw=6, color=self.color, zorder=100))

    # Assuming theta is given in radients
    def update(self, theta):
        cnst = 0.25
        x = self.position[0]+self.radius*np.sin(theta-1/3*np.pi-cnst)
        y = self.position[0]+self.radius*np.cos(theta-1/3*np.pi-cnst)
        self.line_left.set_data([self.position[0],x],[self.position[1],y])

        x = self.position[0]+self.radius*np.sin(theta+1/3*np.pi+cnst)
        y = self.position[0]+self.radius*np.cos(theta+1/3*np.pi+cnst)
        self.line_right.set_data([self.position[0],x],[self.position[1],y])

        x = self.position[0]+self.radius*np.sin(theta+np.pi)
        y = self.position[0]+self.radius*np.cos(theta+np.pi)
        self.line_upper.set_data([self.position[0],x],[self.position[1],y])

        x = self.position[0]+self.radius*np.sin(theta)
        y = self.position[0]+self.radius*np.cos(theta)
        self.circle_outer.set_center((x,y))
