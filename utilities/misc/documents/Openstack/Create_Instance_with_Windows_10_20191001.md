<!-- (c) https://github.com/MontiCore/monticore -->
## Create Instance from image Windows_10_20191001
- Openstack Instance Creation:
    - Source: Create New Volume: No
    - Flavour: min m1.small
    - Networks: external-lehre
    - Security Groups: Windows_RemoteDesktop
    - Key Pair: Create one
- RemoteDesktop:  
    - Username:
        - [DESKTOP-NAME]\admin
            - DESKTOP-DJNIBRO\admin  
    - Password with:
        - nova get-password <instance> [<ssh_private_key_path>]  
            or  
        - openstack dashboard / Instances / Instance Drop-Down -> Retrieve Password / Private Key File
    