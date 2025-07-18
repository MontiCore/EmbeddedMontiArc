import h5py
import logging
import numpy as np
import os
import time
import sys
import shutil
import pathlib
import subprocess

this_directory = pathlib.Path(__file__).parent.resolve()
project_root = os.path.join(this_directory, '..', '..', '..')
input_file = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'files', 'Input.csv')
input_preprocessing = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'preprocessing', 'raw', 'Input.csv')
final_file = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'files', 'Lattice_Structures', 'endfile')
processed_file = os.path.join(this_directory, '..', '..', '..', 'toolchain', 'preprocessing', 'h5', 'raw', 'train.h5')

log_folder = os.environ['LOG_FOLDER']
# Initialize logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
file_handler = logging.FileHandler(f'{log_folder}/rosgym.log')
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

time_to_wait = 1200 # Wait up to 20 minutes

class DynaWrapper(object):
    def __init__(self, material):
        self.material = material
        self.limit = 133 # how many values should be considered from the whole simulation
        self.force = np.zeros(self.limit)
        self.distance = np.zeros(self.limit)

    def simulate(self):
        logger.debug("Dyna simulation started")
        x,y,z,rx,ry,rz = self.material
        stringlist = [str(x),str(y),str(z),str(rx),str(ry),str(rz)]
        materialString = ",".join(stringlist)
        with open(input_preprocessing, 'w') as csvFile:
            csvFile.write(r'1,100,0,' + materialString)
        logger.debug("Written to preprocessing: %s", materialString)
        shutil.copyfile(input_preprocessing, input_file)
        logger.debug("Copied Input.csv to files")
        logger.debug("Wait for local PC to download Input.csv and upload 1.k...")
        # The csv file will be automatically copied to the local PC, processed by nTopology, and copied to the cluster

        # LSDyna will automatically process the file created by nTopology

        # Wait for dyna to finish file
        time_counter = 0
        while not os.path.exists(final_file):
            time.sleep(1)
            time_counter += 1
            if time_counter > time_to_wait:
                logger.critical("LSDyna timed out")
                sys.exit("LSDyna timed out!")
        logger.debug("Wrapper received dyna output")
        logger.debug("Start preprocessing...")
        subprocess.run(['make', 'preprocessing'], cwd=project_root)
        logger.debug("Finished preprocessing")
        file = h5py.File(processed_file, 'r')
        force_all = file["fd_label"][0][1] # force
        displacement_all = file["fd_label"][0][0] # displacement
        self.force = np.array(force_all[:self.limit])
        self.distance = np.array(displacement_all[:self.limit])
        #logger.info("Force is: %s", str(self.force))
        #logger.info("Distance is: %s", str(self.distance))
        logger.debug("Clean up files")
        subprocess.run(['make', 'clean'], cwd=project_root)

    def integrate(self):
        logger.debug("Calculate Integrate")
        return np.trapz(self.force, self.distance)

    def getPeak(self):
        logger.debug("Calculate peak")
        return np.amax(self.force)

    def getPeakPos(self): # starts at 0!!!
        logger.debug("Calculate Peak position")
        return np.argmax(self.force)

    def getFinalLevel(self):
        peak_pos = self.getPeakPos()
        logger.debug("Final level with Peak position: %s", str(peak_pos))
        reduced = self.force[peak_pos:]
        return np.average(reduced)

    def updateMaterial(self, material):
        logger.info("Updated structure: %s", str(material))
        self.material = material
