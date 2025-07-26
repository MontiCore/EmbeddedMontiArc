# TopologyOptimizer


## Description
RL model using EMADL that can optimize lattice structures. Structures are generated using nTopology and evaluated with simulations in LSDyna. The whole process has been optimized to run mainly on the HPC cluster of RWTH Aachen University, although it is technically possible to extend the main executable to run everything locally.

## Requirements

- A PC with Windows 10/11, capable of running nTopology ([System Requirements](https://support.ntopology.com/hc/en-us/articles/360061698333-System-Requirements-Guide))
- nTopology (on local PC)
- anaconda (see https://www.anaconda.com/products/distribution)
- (ideally) Access to a Linux instance on the RWTH HPC Cluster
- If running everything locally:
    - Docker for Windows installed (Requires WSL2 to be enabled!)
    - ANSYS LS-DYNA
- If using the cluster:
    - WinSCP (installed on the local PC)

## Installation

The toolchain, driving the Reinforcement Learning, is split into two parts. A local part, which runs nTopology on a local PC running Windows 10/11, and a remote part which runs everything else on the RWTH HPC Cluster.
    
1. Clone the repository
        
        git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/topologyoptimizer.git
        cd topologyoptimizer
2. To prevent any dependency issues we create an own environment for the project via conda

        conda create -n topologyoptimizer python=3.9 pip
3. Now activate the environment (In the future this must be done always before you run anything in this project)

        conda activate topologyoptimizer
4. Create the `config.ini` file by copying `config.ini.template`and filling it with your settings. For information on what the specific values mean look at section
   [Configuration](#configuration)

        #for wsl or windows powershell
        cp config.ini.template config.ini
        #for windows command line
        copy config.ini.template config.ini
5. Install python dependencies
    
        pip install -r requirements.txt
6. Create ssh keys for connecting to the cluster, see section [SSH Keys](#ssh-keys)


## Configuration

    ClusterHost:                hostname of the hpc cluster
    ClusterUser:                username on the hpc cluster
    ClusterKeyPassword:         passphrase for ssh key
    ClusterWorkingDirectory:    Absolute path to the working directory on the cluster
    ClusterLogFolder:           Absolute path to the log directory on the cluster
    ClusterProjectAccount:      project account to use for sbatch jobs
    SSHKeyFolder:               Absolute path to the ssh keys on the local machine
    LocalLogFolder:             Absolute path to the log folder on the local machine
    ntopcl:                     Absolute path to the ntopcl executable

## SSH Keys
You need to generate an ssh key (Ed25519), to be able to connect to the hpc cluster:
1. Follow the steps in [here](https://www.simplified.guide/putty/puttygen-generate-ssh-key-pair) to create a private and public key in the openssh format. Make sure to enter a passphrase for the key and name the two files `key.priv` (Private OpenSSH Key) and `key.pub` (Public OpenSSH Key).
2. Follow the steps in [here](https://www.simplified.guide/putty/convert-ppk-to-ssh-key) to to convert the private key to the Putty Private Key format. Name this file `key.ppk`
3. Put all 3 key files into a folder of your choice and put the folder location into the `config.ini` in the `SSHKeyFolder` key
4. Add the public key to the `~/.ssh/authorized_keys` file in your home directory on the cluster:

        #in wsl
        ssh-copy-id -i <path-to-public-key> <your-cluster-user>@login18-1.hpc.itc.rwth-aachen.de

## <em>TODO</em> Local Training
If you are running everything locally, you need to download the docker container for the preprocessing. You can download the docker container from `registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/dgl-qd`. Please name the container "preprocessing".

**Note that the Machine Learning component has not been adapted for local execution and will not be executed locally!**

Depending on whether the FEM simulation and preprocessing should run on the cluster or locally, you need to change a few parameters:

### Only necessary for local execution

- In the `runcommand.bat` file at `files/Lattice_Structures`, please adjust the paths to the LS-DYNA executable and the `main.key` file in that same folder. Also adjust the cpu and memory limits if needed, depending on the capabilities of your PC and your LS-DYNA license.
- In the `container.py` file in the local `tools` folder, you need to adjust the path at line 14 to match your path to the `preprocessing` folder.

## Usage

    Usage: python main.py [options] COMMAND
    Commands:
      install   install the remote part of this software on the hpc cluster
      train     start the training of the agent
      execute   execute the current agent


    Options:
      -h, --help   show this help message and exit
      -l, --local  execute agent on the local pc instead of the cluster

### <em>TODO</em> Local execution (only toolchain, no Machine Learning!)

First, place an input file called `Input.csv` in the `files` folder. The explanation for the columns of the `.csv` file can be found below in the **Technicalities** section.

Please make sure that Docker is running in the background before running the toolchain.

To start the toolchain, simply run `main-local.py` from your local machine. The toolchain will ask you for the number of iterations you want to run the toolchain. However, as the Reinforcement Learning component has not been adapted for local execution, any number over 1 will end up in an error, since no new input file is generated.

### Cluster execution

Please make sure you are connected to the RWTH network!

Also, before execution, please make sure that all shell-files (`*.sh`) are executable (execute `chmod +x <filename>` to make a file executable). This problem occurs because git is unable to track file permissions across both Windows and Linux at the same time.

To start the training, simply run 
    
    python topologyoptimizer/main.py train
in the root folder of the project. The toolchain will then start the execution of the toolchain and the agent-training on the cluster. After the training has finished on the cluster (training parameters can be adjusted in `cluster/rl/model/topology/agent/network/TopoQNet.conf`), the local toolchain will time-out and stop. To stop the local toolchain manually before the time out, CTRL-C can be pressed. Note that this will **not** stop the training job running on the cluster!

After the training, you can run 
    
    python topologyoptimizer/main.py execute
to execute the agent. The results, returned by the agent, can be seen in the log files on the cluster, and are also printed out on the local PC.

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
