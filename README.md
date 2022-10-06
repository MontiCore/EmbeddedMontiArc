# TopologyOptimizer


## Description
RL model using EMADL that can optimize lattice structures. Structures are generated using nTopology and evaluated with simulations in LSDyna. The whole process has been optimized to run mainly on the HPC cluster of RWTH Aachen University, although it is technically possible to extend the main executable to run everything locally.

## Requirements

- A PC with Windows 10/11, capable of running nTopology ([System Requirements](https://support.ntopology.com/hc/en-us/articles/360061698333-System-Requirements-Guide))
- nTopology (on local PC)
- Python 3.9.x installed on local PC (3.10+ does not work for local execution of the entire toolchain! It may work for cluster-execution but has not been tested!) as well as the following dependencies: `certifi`, `charset-normalizer`, `idna`, `pip`, `pywin32`, `requests`, `setuptools`, `urllib3`, `websocket-client`
- (ideally) Access to a Linux instance on the RWTH HPC Cluster
- If running everything locally:
    - Docker for Windows installed (Requires WSL2 to be enabled!)
    - ANSYS LS-DYNA
    - Additional python dependencies: `docker`
- If using the cluster:
    - WinSCP (installed on the local PC)
    - Additional python dependencies (on the local PC): `bcrypt`, `cffi`, `cryptography`, `paramiko`, `PyNaCl`, `six`, `pycparser`

## Installation

The toolchain, driving the Reinforcement Learning, is split into two parts. A local part, which runs nTopology on a local PC running Windows 10/11, and a remote part which runs everything else on the RWTH HPC Cluster.

To install, please clone this repository, then copy the content of `/src/main/local/` to a local folder of your choice, and the content of `/src/main/cluster/` to the compute cluster. If possible, place the content of `/src/main/cluster/` in your `$HOME/Dokumente/topologyoptimizer` folder, as some scripts used for the preprocessing are designed to use this path. Otherwise, you may have to adjust the paths specified in some of the files.

If you plan to use the cluster, you need to generate an ssh key for authentication (Ed25519). Place the key in the `tools/Keys` folder on your local PC, and make sure to secure your key with a password. The key is needed in both `.ppk` and split `.priv` and `.pub` format. Guides on how to generate these files can be found [here](https://www.simplified.guide/putty/puttygen-generate-ssh-key-pair) and [here](https://www.simplified.guide/putty/convert-ppk-to-ssh-key). Please name your files `key.ppk` (PuTTY format), `key.priv` (Private OpenSSH Key) and `key.pub` (Public OpenSSH Key). You will also need to add the public key to the authorized keys on the cluster.

If you are running everything locally, you need to build the docker container for the preprocessing. You can use the Dockerfile and requirements file in the `/additional_files/Docker` folder in the root of this repository. Please name the container "preprocessing" (command: `docker build -t preprocessing .`).

**Note that the Machine Learning component has not been adapted for local execution and will not be executed locally!**

Depending on whether the FEM simulation and preprocessing should run on the cluster or locally, you need to change the following parameters:

The following files are in the local `tools` folder:

- In the `errorscript_Loop.py` file, please  adjust the path at line 25
- In the `Runiterations_AOB_8_Table_3.py` file, please adjust the path variables at the beginning of the file so match your nTopology installation.
- Before the first execution, please open the `Cube_Infill_Lattice_V3_for_DoE.ntop` file in nTopology and adjust the export path for the output file to match your local username and folder structure (the parameter can be found in section 1 under "Path_part_1".

### Only necessary for local execution

- In the `runcommand.bat` file at `files/Lattice_Structures`, please adjust the paths to the LS-DYNA executable and the `main.key` file in that same folder. Also adjust the cpu and memory limits if needed, depending on the capabilities of your PC and your LS-DYNA license.
- In the `container.py` file in the local `tools` folder, you need to adjust the path at line 14 to match your path to the `preprocessing` folder.

### Only necessary for cluster execution

- In the `main-cluster.py` file at the root of the local folder, you need to adjust line 13 to your cluster username, line 14 to your private ssh key's password, as well as lines 46 and 76 to the respective paths on the cluster.
- In the `cluster-agent.py` file at the root of the local folder, you need to adjust line 9 to your cluster username, line 10 to your private ssh key's password, as well as lines 31, 43 and 73 to the respective paths on the cluster.
- In the `cluster.py` file in the local `tools` folder, you need to adjust the path to the root folder of the project on your cluster account in line 15.
- In the `agent.py`file in the local `tools` folder, you need to adjust the path to the root folder of the project on your cluster account in line 15.
- In the `run.job` file at the root of the cluster folder, you need to adjust your account and log-path.
- In the `cleanup_preprocessing.py` file in the `toolchain/preprocessing` folder on the cluster, you need to adjust the cluster project root path in line 6.
- In the `copy_preprocessing.py` file in the `toolchain/preprocessing` folder on the cluster, line 6 needs to be adjusted to reflect the project root path.

## Usage

### Local execution (only toolchain, no Machine Learning!)

First, place an input file called `Input.csv` in the `files` folder. The explanation for the columns of the `.csv` file can be found below in the **Technicalities** section.

Please make sure that Docker is running in the background before running the toolchain.

To start the toolchain, simply run `main-local.py` from your local machine. The toolchain will ask you for the number of iterations you want to run the toolchain. However, as the Reinforcement Learning component has not been adapted for local execution, any number over 1 will end up in an error, since no new input file is generated.

### Cluster execution

Please make sure you are connected to the RWTH network before running any script!

Also, before execution, please make sure that all shell-files (`*.sh`) are executable (execute `chmod +x <filename>` to make a file executable). This problem occurs because git is unable to track file permissions across both Windows and Linux at the same time.

To start the toolchain, simply run `main-cluster.py` from your local machine.

The toolchain will then start the execution of the toolchain and the agent-training on the cluster. After the training has finished on the cluster (training parameters can be adjusted in `cluster/rl/model/topology/agent/network/TopoQNet.conf`), the local toolchain will time-out and stop. To stop the local toolchain manually before the time out, CTRL-C can be pressed. Note that this will **not** stop the training job running on the cluster!

After the training, you can use the `cluster-agent.py` script in the local folder, to execute the agent. The results, returned by the agent, can be seen in the log files on the cluster, and are also printed out on the local PC.

### Technicalities

#### Definition of the input file

The toolchain and Reinforcement Learning components use a `.csv` file to define a lattice structure. The columns of this file are defined as follows:

- **Iteration**: Number to identify structures. Should be left at 1, as there is always only one structure generated per execution of nTopology.
- **Size**: Size of the lattice structure, predefined as 100 mm. The agent does not change the size throughout the training.
- **Unit_type_scalar**: Base structure/type of lattice (see below) The agent learns this parameter during training. This parameter has integer values between 0 and 6 where each value represents one of the following base structures (in order): Simple cubic, Diamond, Fluorite, Octet, Truncated cube, Truncated octahedron, Kelvin cell.
- **scale (x-axis)**: Lattice scaling on the x-axis. The agent learns this parameter during training.
- **scale (y-axis)**: Lattice scaling on the y-axis. The agent learns this parameter during training.
- **scale (z-axis)**: Lattice scaling on the z-axis. The agent learns this parameter during training.
- **rotation (x-axis)**: Lattice x-axis rotation. The agent learns this parameter during training.
- **rotation (y-axis)**: Lattice y-axis rotation. The agent learns this parameter during training.
- **rotation (z-axis)**: Lattice z-axis rotation. The agent learns this parameter during training.

#### Known issues

Due to a bug, sometimes ROS is not initialized correctly on the cluster. This can lead to an error when trying to execute the agent. If this happens, the following command needs to be executed before starting the agent (add this line after the shebang in `rl/execute.sh` on the cluster):
```bash
. /opt/ros/noetic/setup.sh
```
