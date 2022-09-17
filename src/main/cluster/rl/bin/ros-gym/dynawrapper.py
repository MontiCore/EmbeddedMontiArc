import h5py
import numpy as np
import os
import time
import sys
import pathlib
import subprocess

this_directory = pathlib.Path(__file__).parent.resolve()
project_root = os.path.join(this_directory, '..', '..', '..')
input_file = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'files', 'Input.csv')
final_file = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'files', 'Lattice_Structures', 'endfile')
processed_file = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'preprocessing', 'h5', 'raw', 'train.h5')

time_to_wait = 120 # Wait up to 2 minutes

class DynaWrapper(object):
    def __init__(self, material):
        self.material = material
        self.limit = 133 # how many values should be considered from the whole simulation
        self.force = np.zeros(self.limit)
        self.distance = np.zeros(self.limit)

    def simulate(self):
        x,y,z,rx,ry,rz = self.material
        stringlist = [str(x),str(y),str(z),str(rx),str(ry),str(rz)]
        materialString = ",".join(stringlist)
        with open(input_file, 'w') as csvFile:
            csvFile.write(r'1,100,0,' + materialString)

        # The csv file will be automatically copied to the local PC, processed by nTopology, and copied to the cluster

        # LSDyna will automatically process the file created by nTopology

        # Wait for dyna to finish file
        time_counter = 0
        while not os.path.exists(final_file):
            time.sleep(1)
            time_counter += 1
            if time_counter > time_to_wait:
                sys.exit("LSDyna timed out!")

        subprocess.run(['make', 'preprocessing'], cwd=project_root)
        file = h5py.File(processed_file, 'r')
        force_all = file["fd_label"][0][1] # force
        displacement_all = file["fd_label"][0][0] # displacement
        self.force = np.array(force_all[:self.limit])
        self.distance = np.array(displacement_all[:self.limit])
        subprocess.run(['make', 'clean'], cwd=project_root)

    def integrate(self):
        return np.trapz(self.force, self.distance)

    def getPeak(self):
        return np.amax(self.force)

    def getPeakPos(self): # starts at 0!!!
        return np.argmax(self.force)

    def getFinalLevel(self):
        peak_pos = self.getPeakPos()
        reduced = self.force[peak_pos:]
        return np.average(reduced)
