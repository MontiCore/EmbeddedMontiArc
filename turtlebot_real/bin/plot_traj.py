import matplotlib.pyplot as plt
import random
import os
from calc_rmse import import_from_excel

def plot_trajectory(recorded_x, recorded_y, actual_x, actual_y, pdf_file):
    """
    Plot the recorded and actual trajectories, along with the initial and goal positions.

    Parameters:
    - recorded_x (list): x coordinates of the recorded trajectory
    - recorded_y (list): y coordinates of the recorded trajectory
    - actual_x (list): x coordinates of the actual trajectory
    - actual_y (list): y coordinates of the actual trajectory
    - pdf_file (string): the path and file name of the pdf file
    """
    # Plot the recorded trajectory
    plt.plot(recorded_x, recorded_y, label='Recorded path')

    # Plot the actual trajectory
    plt.plot(actual_x, actual_y, label='Actual path')

    # Plot the initial position
    plt.scatter(actual_x[0], actual_y[0], color='red', label='Initial position')

    # Plot the goal position
    plt.scatter(actual_x[-1], actual_y[-1], color='green', label='Goal position')

    # Add axis labels and a legend
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()

    # Save the plot to a pdf file
    plt.savefig(pdf_file)
    print("The plot has been saved to", pdf_file)
    
    
if __name__ == "__main__":

    actual_x, actual_y = import_from_excel('../excel_traj_files/act_traj_exp1.xlsx')
    recorded_x, recorded_y = import_from_excel('../excel_traj_files/rec_traj_exp1.xlsx')
    
    current_directory = os.getcwd()
    pdf_file = current_directory + '/trajectory_exp1.pdf'
    plot_trajectory(recorded_x, recorded_y, actual_x, actual_y, pdf_file)
