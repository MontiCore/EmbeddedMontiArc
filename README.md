# TopologyOptimizer


## Description
RL model using EMADL that can optimize lattice structures. Structures are generated using nTopology and evaluated with simulations in LSDyna. The whole process has been optimized to run mainly on the HPC cluster of RWTH Aachen University, although it is possible to extend the main exectable to run everything locally.

## Requirements

- A PC with Windows 10, capable of running nTopology ([System Requirements](https://support.ntopology.com/hc/en-us/articles/360061698333-System-Requirements-Guide))
- nTopology (on local PC)
- Python 3.9.x installed on local PC (does not work with 3.10+!) as well as the following dependencies: `certifi`, `charset-normalizer`, `idna`, `pip`, `pywin32`, `requests`, `setuptools`, `urllib3`, `websocket-client`
- (ideally) Access to a linux instance on the RWTH HPC Cluster
- If running everything locally:
    - Docker for Windows installed (Requires WSL2 to be enabled!)
    - ANSYS LS-DYNA
    - Additional python dependencies: `docker`
- If using the cluster:
    - WinSCP (installed on the local PC)
    - Additional python dependencies: `bcrypt`, `cffi`, `cryptography`, `paramiko`, `PyNaCl`, `six`, `pycparser`

## Installation

The toolchain driving the Reinforcement Learning, is split in two parts. A local part, which runs nTopology on a local PC running Windows 10, and a remote part which runs everything else on the RWTH HPC Cluster.

To install, please clone this repository, then copy the content of `/src/main/local/` to a local folder of your choice, and the content of `/src/main/cluster/` to the compute cluster. If possible, place the content of `/src/main/cluster/` in your `$HOME/Dokumente/topologyoptimizer` folder, as some of the scripts used for the preprocessing are designed to use this path. Otherwise, you may have to adjust the paths specified in some of the files.

If you plan to use the cluster, you need to generate an ssh key for authentication (Ed25519). Place the key in the tools/Keys folder on your local PC and make sure to secure your key with a password. The key is needed in both `.ppk` and split `.priv` and `.pub` format. Guides on how to generate these files can be found [here](https://www.simplified.guide/putty/puttygen-generate-ssh-key-pair) and [here](https://www.simplified.guide/putty/convert-ppk-to-ssh-key). Please name your files `key.ppk` (PuTTY format), `key.priv` (Private OpenSSH Key) and `key.pub` (Public OpenSSH Key). You will also need to add the public key to the authorized keys on the cluster.

Furthermore, if you are running everything locally, you need to build the docker container for the preprocessing. You can use the Dockerfile and requirements file in the `/additional_files/Docker` folder in the root of this repository. Please name the container "preprocessing".

Depending on whether the FEM simulation and preprocessing should run on the cluster or locally, you need to change the following parameters:

**Note that the Machine Learning component has not been adapted for local execution and will not be executed by default!**

All these files are in the `tools` folder:

- In the `errorscript_Loop.py`file, please  adjust the path at line 25
- In the `Runiterations_AOB_8_Table_3.py` file, please adjust the path variable at the beginning of the file so match your nTopology installation.

### Local execution

- In the `container.py` file in the `tools` folder, you need to adjust the path at line 14 to match your path to the `preprocessing` folder.
- In the `runncommand.bat` file at `files/Lattice_Structures`, please adjust the paths to LS-DYNA and the `main.key` file in that same folder. Also adjust the cpu and memory limits if needed, depending on the capabilities of your PC and your LS-DYNA license.

### Cluster execution

#### Changes for local files:

#### Changes for files on the cluster:

## Usage

If you are using the cluster, please make sure to be connected to the RWTH network before running the scripts.

To start the toolchain, simply run `main.py` from your local machine. The toolchain will ask you for the number of iterations you want to train the network and if you want to execute everything except nTopology locally or on the compute cluster.
