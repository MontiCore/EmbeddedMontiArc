# Development Setup

## Docker Image Setup

All the scripts and processes of this repository can be performed in the VM of the DSA (if this repository is cloned inside). But you can also setup the build image locally:

- Install Docker (Make sure it is in your PATH for Windows)
- Extract the image yourself from the VM
- Or download it

### Extracting the image from the VM

Use the `docker save` command, extract the image from the VM disk, load it with `docker load`.

### Downloading it

- Download the image [here](https://gigamove.rz.rwth-aachen.de/d/id/GEqRXufeFUSu6C?7&id=GEqRXufeFUSu6C).
- Unzip it to `latest.tar`
- Run the following:

```bash
docker load -i latest.tar
# Then add a tag for easier referencing
docker tag <image_id> dsa-ddc:latest
# Where <image_id> is the sha5 output from the 'docker load' command (Loaded image ID: sha256:...)
```

## SSH setup

The connection to the VCG goes through SSH, and you must be in the RWTH-network (through the [RWTH VPN](https://www.ip.rwth-aachen.de/vpn/)).

```bash
ssh student@lablogin.se.rwth-aachen.de
# Password: segue284!simulative
```

In order connect more easily, you can setup a ssh key and send it to the Raspberry-Pi.

```bash
# To generate a local key (Look online for the details of this command)
ssh-keygen
# To send it to the Raspberry Pi
ssh-copy-id -i <id_file> student@lablogin.se.rwth-aachen.de
# The default generated <id_file> is ~/.ssh/id_rsa
```

You can also setup an SSH profile for shorter command lines. For this add the following to your *SSH config file* (default: `~/.ssh/config`):

```bash
Host lab
        HostName lablogin.se.rwth-aachen.de
        User student
        IdentityFile ~/.ssh/id_rsa
```

`lab` can be anything you want and IdentityFile must point to your generated key. Then all ssh/scp calls work by just giving them `lab` (or the name you chose).

```bash
# Example
ssh lab
scp some_file.txt lab:/remote/path/to/some_file.txt
```

## `build_environment` scripts config

The `build_environment/config` file contains a series of entries used in the different scripts. Make sure all of them are set to your needs. If you use the file structure of this repository and have tagged your build image as specified above, you shouldn't have to change any of these.

- `IMAGE_NAME`: The name or id of the DSA build image (default: `dsa-ddc:latest` if you used the command above).
- `IMAGE_WORKING_DIRECTORY`: Where the `build_environment` folder will be available in the build container. (Also the working directory on container startup with the `start.bat/.sh` script.)
- `EXTERNALS_DIRECTORY`: The folder containing the `shared_cpp` folder, the `libddc` folder (needed if building with libddc) and the `armadillo` folder (if building EMA models).
- `IMAGE_EXTERNALS_DIRECTORY`: Where the `externals` folder is available inside the build container.
- `SSH_CONFIG`: the name used to connect in ssh/scp calls. Can be left as `student@lablogin.se.rwth-aachen.de` if you have an SSH profile set up.

## EMA compiling config

If compiling EMA projects with EMADL2CPP or EMAM2CPP, make sure you:

- Clone the project [EMADL2CPP](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP) or [EMAM2CPP](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp).
- Build it locally (`mvn install ...`).
- Set the appropriate paths/versions in the `config` file along the scripts.
