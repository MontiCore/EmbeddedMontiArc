from ast import arg
import logging
from pathlib import Path
from subprocess import Popen, run, PIPE, STDOUT
import csv
import sys
from config import config

# Initialize logging
logger = logging.getLogger(__name__)

def generate_lattice(input_csv_file: Path, output_file: Path):
    logger.debug("Start function generate_lattice")
    ntopcl = config.get("DEFAULT","ntopcl")
    resources_dir = config.get("DEFAULT", "ResourcesDirectory")
    script_file = Path(resources_dir).joinpath("Cube_Infill_Lattice_V3_for_DoE.ntop")
    
    parameters_list = []
    logger.debug("Read csv file")
    with open(input_csv_file) as csvData:
        csvReader = csv.reader(csvData)
        for row in csvReader:
            parameters_list.append({
                "iteration": float(row[0]),
                "size": float(row[1]),
                "unittype": float(row[2]),
                "Scale_x": float(row[3]),
                "Scale_y": float(row[4]),
                "Scale_z": float(row[5]),
                "Rotation_x": float(row[6]),
                "Rotation_y": float(row[7]),
                "Rotation_z": float(row[8])
            })
        #bcc=1,fcc=0 (others can be added/changed in ntop file)

    for p in parameters_list:
        arguments = f'{ntopcl} \
                        -i {p["size"]:0.1f}mm \
                        -i {p["unittype"]:0.1f} \
                        -i {p["Scale_x"]:0.1f}mm \
                        -i {p["Scale_y"]:0.1f}mm \
                        -i {p["Scale_z"]:0.1f}mm \
                        -i {p["Rotation_x"]:0.1f}deg \
                        -i {p["Rotation_y"]:0.1f}deg \
                        -i {p["Rotation_z"]:0.1f}deg \
                        -i {output_file} \
                        {script_file}'

        process = run(arguments,shell=False, stdout=PIPE, stderr=STDOUT, close_fds=True, encoding="utf8",)
        if process.returncode != 0:
            print(f'command "{arguments}" failed with errors:')
            print(process.stdout)
            logger.critical("NTOP failed")
            sys.exit(1)

        post_process(output_file)
    logger.debug("Finished function generate_lattice")

def post_process(file: Path):
    logger.debug("Start function post_process")
    with open(file, 'r') as fd:       
        data = fd.read()
    
    data = data.replace('C***', '$***') 
    
    with open(file, 'w') as fd:
        fd.write(data)
    logger.debug("Finished function post_process")