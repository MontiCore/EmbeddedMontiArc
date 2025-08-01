<!-- (c) https://github.com/MontiCore/monticore -->
# Preparation steps for Ubuntu 18.04 LTS

## First start
- connect via putty with `ubuntu@ipadress` and your ssh-key
- `sudo apt update`
- `sudo apt upgrade`
- `passwd ubuntu`  
    enter a new password

## Gnome Desktop
- Also see [Gnome Desktop](https://linuxconfig.org/how-to-install-gnome-on-ubuntu-18-04-bionic-beaver-linux)  
- `sudo apt install gnome-session gdm3`
- `sudo apt install tasksel`
- `sudo tasksel install ubuntu-desktop`
- `sudo service gdm3 start`

## XRDP
- Also see [XRDP](https://www.hiroom2.com/2018/04/29/ubuntu-1804-xrdp-gnome-en/)
- `sudo apt install -y xrdp`
- `sudo sed -e 's/^new_cursors=true/new_cursors=false/g' -i /etc/xrdp/xrdp.ini`
- `D=/usr/share/ubuntu:/usr/local/share:/usr/share:/var/lib/snapd/desktop`
- `cat <<EOF > ~/.xsessionrc  
  export GNOME_SHELL_SESSION_MODE=ubuntu  
  export XDG_CURRENT_DESKTOP=ubuntu:GNOME  
  export XDG_DATA_DIRS=${D}  
  export XDG_CONFIG_DIRS=/etc/xdg/xdg-ubuntu:/etc/xdg  
  EOF`
- `cat <<EOF | \
    sudo tee /etc/polkit-1/localauthority/50-local.d/xrdp-color-manager.pkla
  [Netowrkmanager]
  Identity=unix-user:*
  Action=org.freedesktop.color-manager.create-device
  ResultAny=no
  ResultInactive=no
  ResultActive=yes
  EOF`
- `sudo systemctl restart polkit`

## Connect
- Windows Remote Desktop
    - user: ubuntu
    - password: your password
    
## Install Docker 
- Also see [Docker for Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
- `sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common`
- `curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -`
- `sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"`
- `sudo apt update`
- `sudo apt install docker-ce docker-ce-cli containerd.io`
- Test with `sudo docker run hello-world`

## Install Gitlab-Runner
- Also see [Gitlab-Runner](https://docs.gitlab.com/runner/install/linux-repository.html)
- curl -L https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh | sudo bash
- sudo apt-get install gitlab-runner