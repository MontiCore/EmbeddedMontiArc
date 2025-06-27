<!-- (c) https://github.com/MontiCore/monticore -->
# EMAM2MQTT

A project that generates MQTT-Adapter for EmbeddedMontiArcMath components.

## Warning

This generator is part of an composite generator and does not create an executable. Look at EMAM2Middleware if you want to generate one.

## Getting Started

These instructions will get this project installed on your **Ubuntu** system

### Installing project

[Download](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/emam2mqtt/-/archive/master/emam2mqtt-master.zip) this project to your computer

[Download](https://openjdk.java.net/install/) and install Java JDK (alternatively via `apt install openjdk-8-jdk`)

[Download](https://maven.apache.org/guides/getting-started/) and install Maven (alternatively via `apt install maven`)

[Download](https://www.eclipse.org/downloads/) and install Ecplise IDE

Import the project as "Maven Project" in Eclipse

Open terminal and switch to the root of your project directory

In terminal run

`mvn clean install -s settings.xml`

### Installing MQTT broker (locally on Ubuntu)

Open terminal

Add new MQTT repository

`apt-add-repository ppa:mosquitto-dev/mosquitto-ppa`

Run update

`apt-get update`

Install mosquitto and mosquitto-clients

`apt install mosquitto`

`apt install mosquitto-clients`

Start the Mosquitto Brocker

`/etc/init.d/mosquitto start`

### Installing MQTT broker (locally on Windows)

[Download](https://mosquitto.org/files/binary/win64/mosquitto-1.6.2-install-windows-x64.exe) and install Mosquitto

Restart your computer

Mosquitto is now available and can be accessed via command line using `mosquitto -h`

### Installing MQTT libraries (locally on Ubuntu)

Open terminal and run

`apt install build-essential gcc make cmake git`

#### Installing PahoMQTT C library
run
```bash
git clone https://github.com/eclipse/paho.mqtt.c
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF
sudo cmake --build build/ --target install
```
#### Installing PahoMQTT C++ library
**after** installation of C library run
```bash
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF
sudo cmake --build build/ --target install
```
[Documentation](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/emam2mqtt/-/archive/master/emam2mqtt-master.zip) of the PahoMqtt C++ library
#### Set environment variables for MQTT (optional)
additionaly, you can set the environment variables for MQTT libs and includes directory by modifying the environment file on your system
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

## Tagging for Mqtt
Mqtt only needs an Topic to publish or subscribe to `{ topic = {topic} }`

For example `{topic = /echo}` for topic "/echo"

[Here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/EmbeddedMontiArc/tree/master/src/test/resources/middleware/mqtt) an Example for a Mqtt Tagging File
