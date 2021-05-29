# (c) https://github.com/MontiCore/monticore  
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from Player import Player
from WheelFig import WheelFig
from AccuracyFig import AccuracyFig

def create_fig():

    fig = plt.figure()

    ax_player = fig.add_subplot(10,1,1)
    ax_image = plt.subplot2grid((10,1),(1,0),rowspan=5, fig=fig)
    ax_real = plt.subplot2grid((10,2),(6,0),rowspan=2, fig=fig, title="Real")
    ax_pred = plt.subplot2grid((10,2),(6,1),rowspan=2, fig=fig, title="Predicted")
    ax_acc = plt.subplot2grid((10,1),(8,0),rowspan=2, fig=fig, title="Error (in degrees)")

    # Image
    ax_image.axis('off')

    # Steering wheels
    ax_real.axis('off')
    ax_real.set_aspect(1)
    ax_pred.axis('off')
    ax_pred.set_aspect(1)

    # Accuracy 1D plot
    ax_acc.get_yaxis().set_visible(False)
    ax_acc.set_frame_on(False)
    ax_acc.set_xlim([-360,360])
    ax_acc.set_xticks(np.arange(-360,361,90))
    return fig, ax_player, ax_image, ax_real, ax_pred, ax_acc

def start_plot(images, targets_real, targets_pred, delay):
    fig, ax_player, ax_image, ax_real, ax_pred, ax_acc = create_fig()

    # Callback
    def update(i):
        sleep(delay/1000)
        image.set_data(images[i])
        wheel_real.update(-targets_real[i])#negative values because predicted angles are inverted
        wheel_pred.update(-targets_pred[i])
        acc.update(-targets_real[i],-targets_pred[i])

    player = Player(fig, ax_player, update, maxi=len(targets_real)-1, interval=50)
    image = ax_image.imshow(images[0])

    center = (0.5,0.5)
    radius = 0.4
    wheel_real = WheelFig(ax_real, center, radius, "green")
    wheel_pred = WheelFig(ax_pred, center, radius, "red")

    acc = AccuracyFig(ax_acc)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("[DEBUG MODE] Running with random values.")
    num_images = 100
    images = np.random.randint(0,255,(num_images,480,640,3))
    targets_real = 2*np.sin(np.linspace(0,5,num_images))+6*np.cos(np.linspace(0,5,num_images))
    targets_pred = targets_real + np.random.normal(0,0.2,(num_images,))

    start_plot(images, targets_real, targets_pred,0)
