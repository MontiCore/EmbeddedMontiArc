import sys
import xlsxwriter
import pandas as pd
import rospy
from nav_msgs.msg import Odometry
from movementMaintainer import getPosition


traj_x, traj_y = [], []

def odom_cb(data):
    global traj_x, traj_y
    odom_arr = getPosition(data)
    print(str(odom_arr))
    traj_x += [odom_arr[0]]
    traj_y +=  [odom_arr[1]]
    
odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)

def export_to_excel(x, y, file_name):
    
    """
    This function takes in two variables x and y as arrays, 
    and exports them to an Excel spreadsheet with column names 'x_values' and 'y_values'
    discarding identical values that are one after the other.
    The values of x and y will be written to the first two columns of the worksheet.
    """
    # Create a new Excel file
    workbook = xlsxwriter.Workbook(file_name+'.xlsx')
    worksheet = workbook.add_worksheet()
    
    # Write the column names to the first row
    worksheet.write(0, 0, 'x_values')
    worksheet.write(0, 1, 'y_values')
    
    x_filtered, y_filtered = [x[0]], [y[0]]
    for i in range(1, len(x)):
        if x[i] != x[i-1] or y[i] != y[i-1]:
            x_filtered.append(x[i])
            y_filtered.append(y[i])

    # Write x and y to the first two columns
    for i, value in enumerate(x_filtered):
        worksheet.write(i+1, 0, value)
    for i, value in enumerate(y_filtered):
        worksheet.write(i+1, 1, value)

    # Save the file
    workbook.close()
    
    
def import_from_excel(file_path):
    """
    This function takes in a file path as an argument and returns the x and y lists
    that were exported using the export_to_excel function.
    """
    # Read the Excel file into a pandas DataFrame
    data = pd.read_excel(file_path)

    # Extract the x and y lists from the DataFrame
    x = data['x_values'].to_list()
    y = data['y_values'].to_list()

    return x, y

#### how to use ####x, y = import_from_excel('variables.xlsx')
    

def my_node(file_name):
    export_to_excel(traj_x, traj_y, file_name)


if __name__ == '__main__':
    rospy.init_node('traj_node')
    param = sys.argv[1]
     
    if param is None:
        raise ValueError("A name for the generated excel file should be passed")
        
    rospy.spin()
    rospy.on_shutdown(my_node(param))

