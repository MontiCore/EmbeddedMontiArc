# -*- coding: utf-8 -*-

import subprocess
from subprocess import Popen, PIPE, STDOUT

import csv
# Here's the input example that needs to be called
# C:\Users\chaseallan\Downloads\nTop-1.6.5-rc\ntop.exe  -i 4.0mm -i C:\Users\chaseallan\Documents\nTop\api\ C:\Users\chaseallan\Documents\nTop\api\sphereApi.ntop


# - This example uses the headless version of our api implementation
#%%------------------------------------------------------------------------------
# Setup the Paths to Necessary Files - User will need to update these files
# based on their machines configuration.
#%%------------------------------------------------------------------------------
#set up path variables
pathToExe = "C:/Program Files/nTopology/nTopology/nTopcl.exe"
pathToFiletoRun = 'C:\\Users\\Anis\\Desktop\\toolchain\\tools\\Cube_Infill_Lattice_V3_for_DoE.ntop'
PathToCSV = 'C:\\Users\\Anis\\Desktop\\toolchain\\files\\Input.csv'
# DER PFAD WIRD IM MODELL DEFINIERT UND HAT IN DIESEM SCRIPT DESHALB KEINE FUNKTION
pathForExport = 'C:\\Users\\Anis\\Desktop\\toolchain\\files\\Lattice_Structures'
#%%----------------------------------------------------------------------------
# Call the API in a Loop
#%%----------------------------------------------------------------------------
#Script
#initialise a list for each input variable 
iteration = []
Size = []
lattice = []
Scale_x = []
Scale_y = []
Scale_z = []
Rotation_x = []
Rotation_y = []
Rotation_z = []

#read csv and fill in list for each iteration 

with open(PathToCSV) as csvData:
    csvReader = csv.reader(csvData)
    for row in csvReader:
        iteration.append(row[0])
        Size.append(row[1])
        lattice.append(row[2])
        Scale_x.append(row[3])
        Scale_y.append(row[4])
        Scale_z.append(row[5])
        Rotation_x.append(row[6])
        Rotation_y.append(row[7])
        Rotation_z.append(row[8])


# run loop through list, 1st element in each list is first iteration and so on. 
for n in range(0,len(Scale_x)):
#    # Prep ntop file call
    realscalex=float(Scale_x[n])
    realscaley=float(Scale_y[n])
    realscalez=float(Scale_z[n])
    realrotationx=float(Rotation_x[n])
    realrotationy=float(Rotation_y[n])
    realrotationz=float(Rotation_z[n])
    realsize=float(Size[n])
    realiteration=float(iteration[n])
    unittype=float(lattice[n])
#bcc=1,fcc=0 (others can be added/changed in ntop file)
    
#notice for arguments -i is input, %0.1f means scalar input, must be in the same order as inputs to ntop file. %s means string input (first and last are not inputcs, but location to file and program (see above)) 
    arguments = ("%s -i %.f -i %0.1fmm -i %0.1f -i %0.1fmm -i %0.1fmm -i %0.1fmm -i %0.1fdeg -i %0.1fdeg -i %0.1fdeg -i %s %s" % (pathToExe,realiteration,realsize,unittype,realscalex,realscaley,realscalez,realrotationx,realrotationy,realrotationz,pathForExport,pathToFiletoRun));
    # Call nTop file
    subprocess.call(arguments);
#below is ntop code for prnting the ntop errors and logs in your environment
    p = Popen(arguments, shell=False, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    for line in p.stdout:
        print(line.rstrip())

