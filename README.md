<!-- (c) https://github.com/MontiCore/monticore -->
# CooperativeIntersection

## Simulator and compiler installation
### Docker(Recommended)
1. Install [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce)
2. Download the image:
```bash
sudo docker pull registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/cooperativeintersection/emam-coincar
```

### Native
Warning: This will take long, since a lot of packages are installed and the simulator will be compiled
1. run `sudo ./installPackages.sh`
2. run `./installLibraries.sh`
    * The installation dir can be changed in the variable `EMA_INSTALL_BASE`(line 6)

Uninstall note: these scripts will add multiple lines to your .bashrc, which you should delete after deleting the downloaded libraries

## Usage
0. (Make changes to the EMAM-Model)
1. Run `sudo ./oneClickDocker.sh`(if you installed using docker) or `./oneClick.sh`(if you installed native) from the root of this Project
    * The project will be generated,
    * compiled,
    * and the simulator will be started
2. In the rqt_reconfigure windows uncheck: time_mngmt > stop_time
3. Observe:
    * the two vehicles will collide the first time they cross the intersection
    * after that the intersection controller is active and tries to prevent vehicle collisions
    * the vehicles will cross the intersection 10 times with different starting velocities and positions
    * every collision is documented in the terminal output
