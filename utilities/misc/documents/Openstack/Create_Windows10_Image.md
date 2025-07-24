<!-- (c) https://github.com/MontiCore/monticore -->
## Windows 10 Image Creation for Openstack on Ubunutu
For additional help see:
    [Link1](https://superuser.openstack.org/articles/how-to-deploy-windows-on-openstack/)
        or
    [Link2](https://docs.openstack.org/image-guide/windows-image.html)

1. Download ISO
    1. Download a Windows10 Image from Microsoft (choose language of choice) [Link](https://www.microsoft.com/de-de/software-download/windows10)
    2. Download [VirtIO Drivers](https://fedorapeople.org/groups/virt/virtio-win/direct-downloads/latest-virtio/virtio-win.iso)   
    3. Move to ~/openstack/iso
2. Setup  
    `mkdir ~/openstack`  
    `mkdir ~/openstack/iso`  
    `mkdir ~/openstack/img`  
    `mkdir ~/openstack/credentials`  
    `sudo apt-get update`
    1. Install qemu-utils:  
        `sudo apt-get install qemu-utils`
    2. [Install Python](https://docs.python-guide.org/starting/install3/linux/)  
        `sudo apt-get install python3.6`
    3. Setup openstack CLI
        1. install the openstack CLI: [Link](https://docs.openstack.org/mitaka/user-guide/common/cli_install_openstack_command_line_clients.html)  
            `sudo apt-get install python-dev python-pip`  
            `pip install python-openstackclient`  
        2. Download the credetial file:
            - In the openstack-dashboard (https://openstack.se.rwth-aachen.de/project/) -> user (top right corner) -> OpenStack RC File v3 
            - Move to ~/openstack/credentials
        3. source the downloaded file:  
                `cd ~/openstack/credentials`  
                `source MontiCar-openrc.sh`  
                - Enter password
        4. Test with: (should list you something)  
                `openstack image list`
        5. If there occur problems, try again from inside eduroam
    4. install virt-manager:  
        `sudo apt-get install virt-manager`
3. Create an empty raw Image (Used to install Windows on. Gets uploaded later)  
    `cd ~/openstack/img`  
    `qemu-img create -f raw MyWindows10.img 15G`
    - 15G should be sufficiant. If there are problems while the Windows installation later with the size, you need to change this value.  
4. Install the Windows image with virt-manager (Also see [Link](https://docs.openstack.org/image-guide/windows-image.html) )
    1. start the virt-manager  
        `virt-manager`
    2. Create a new Connection
    3. Create a new Virtual Machine:
        - Local install Media
        - Use ISO Image: choose the Windows10 image (~/openstack/iso)
        - Uncheck Automatically detect operating system and choose Windows / Microsoft Windows 10
        - Memory: At least 2048 MB
        - CPUs: Choose at least 2
        - Choose select or create custom storage and choose the raw image created (~/openstack/img)
        - Name it
        - Customize configuration:
            - IDE Disk 1:  
                - Disk bus: VirtIO
            - network (NIC:5e:b8:06 or similar):  
                - device model: virtio
            - Add Hardware  
                - Storage  
                    - Device type: CDROM device  
                    - Select or create custom storage:  
                        - Choose the VirtIO drivers (~/openstack/iso)  
        - Begin Installation
    4. Installation
        1. Choose Drivers:  
            - E:\viostor\w10\amd64\  
            - E:\NetKVM\w10\amd64  
        2. install
    5. After Installation
        1. create an User ("Admin")
        2. enable remote-desktop (Search bar: Remote Dekstop)
            1. Save the PC-Name for login Remote-Login
                - [DESKTOP-NAME]
                    - (Something like DEKTOP-DJNIBRO)
        3. Install Cloudbase
            - powershell (as admin)
                `Set-ExecutionPolicy Unrestricted`  
                `Invoke-WebRequest -UseBasicParsing https://cloudbase.it/downloads/CloudbaseInitSetup_Stable_x64.msi -OutFile cloudbaseinit.msi`  
                `\cloudbaseinit.msi`  
            - Username: Admin
            - uncheck metadata password
            - Serial port for logging: COM1
            - Complete the Cloudbase-Init Setup Wizard
                - Run Sysprep
                - Shutdown
                - Finish
    6. Restart the virtual machine once and let it work
    7. shutdown
5. Upload Image (with sourced credentials, see 1.c)  
    `cd ~/openstack/img`  
    `openstack image create --disk-format raw --file MyWindows10.img "name for OpenStack image"`  

6. Openstack Instance Creation:
    - Source: Create New Volume: No
    - Flavour: min m1.small
    - Networks: external-lehre
    - Security Groups: Windows_RemoteDesktop
    - Key Pair: Create one
7. RemoteDesktop:  
    - Username:
        - [DESKTOP-NAME]\admin
            - DESKTOP-DJNIBRO\admin  
    - Password with:
        - nova get-password <instance> [<ssh_private_key_path>]  
            or  
        - openstack dashboard / Instances / Instance Drop-Down -> Retrieve Password / Private Key File
    