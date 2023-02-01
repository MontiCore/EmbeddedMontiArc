import sys
import xlsxwriter
import pandas as pd
import rospy
from nav_msgs.msg import Odometry
from movementMaintainer import getPosition


traj_x, traj_y = [], []

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
    
    x_filtered, y_filtered = [round(x[0], 2)], [round(y[0], 2)]
    for i in range(1, len(x)):
        x_rounded = round(x[i], 2)
        y_rounded = round(y[i], 2)
        if x_rounded != x_filtered[-1] or y_rounded != y_filtered[-1]:
            x_filtered.append(x_rounded)
            y_filtered.append(y_rounded)

    # Write x and y to the first two columns
    for i, value in enumerate(x_filtered):
        worksheet.write(i+1, 0, value)
    for i, value in enumerate(y_filtered):
        worksheet.write(i+1, 1, value)

    # Save the file
    workbook.close()
    
    
def odom_cb(data):
    global traj_x, traj_y
    odom_arr = getPosition(data)
    print(str(odom_arr))
    traj_x += [odom_arr[0]]
    traj_y +=  [odom_arr[1]]

def run_export_node(file_name):
    def wrapper_func():
        export_to_excel(traj_x, traj_y, file_name)
    
    #rospy.init_node('traj_node', anonymous=True)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
    

    rospy.on_shutdown(wrapper_func)
    #rospy.spin()

