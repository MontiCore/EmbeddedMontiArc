# EMAM2MQTT

A project that generates MQTT-Adapter for EmbeddedMontiArcMath components.

## Getting Started

These instructions will get MQTT up and running on your **Ubuntu** system

### Installing project

[Download](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/emam2mqtt/-/archive/8-handwrite-a-mqtt-adapter/emam2mqtt-3-install-mqtt-linux.zip) this project to your computer

[Download](https://maven.apache.org/guides/getting-started/) and install Maven

[Download](https://www.eclipse.org/downloads/) and install Ecplise IDE

Import the project as "Maven Project" in Eclipse

Open terminal and switch to your project directory

In terminal run

`mvn clean install -s settings.xml`

### Installing MQTT broker (locally on Ubuntu)

Open terminal

Add new MQTT repository

`sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa`

Run update

`sudo apt-get update`

Install mosquitto and mosquitto-clients

`sudo apt-get install mosquitto`

`sudo apt-get install mosquitto-clients`

### Installing MQTT broker (locally on Windows)

[Download](https://mosquitto.org/files/binary/win64/mosquitto-1.6.2-install-windows-x64.exe) and install Mosquitto

Restart your computer

Mosquitto is now available and can be accessed via command line using `mosquitto -h`

### Installing MQTT libraries (locally on Ubuntu)

Open terminal and run

`sudo apt-get install build-essential gcc make cmake cmake-gui cmake-curses-gui`

#### Installing PahoMQTT C library
run 
```bash
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
git checkout v1.2.1
cmake -Bbuild -H. -DPAHO_WITH_SSL=ON
sudo cmake --build build/ --target install
sudo ldconfig
```
#### Installing PahoMQTT C++ library
**after** installation of C library run
```bash
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
cmake -Bbuild -H.
sudo cmake --build build/ --target install
```
#### Set environment variables for MQTT
additionaly, you can set the environment variables for MQTT libs and includes directroy by modifying the environment file on your system
```bash
sudo nano /etc/environment
```
in the file, add a new line and write
```bash
MQTT_LIBS = "PATH/to/libs"
MQTT_INCLUDE_DIR = "PATH/to/includes"
```

## Installing MQTT Publisher/Subscriber demo (locally on Ubuntu)
open terminal, switch to **mqtt_demo** directroy inside the project and run
`cmake .`

compile the demo by running
`make`

run the demo afterwards with `./mqtt_demo`

---

additionaly, you can also use `mosquitto_pub` and `mosquitto_sub` commands from mosquitto broker to subscribe/publish messages

for example:
subscribe to topic **/hello** with `mosquitto_sub -t /hello` and publish a message to this topic with 
`mosquitto_pub -t /hello -m "hi, how are you?"`

