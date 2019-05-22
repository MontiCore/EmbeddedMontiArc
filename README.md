# EMAM2MQTT

A project that generates MQTT-Adapters for EmbeddedMontiArcMath components.

## Getting Started

These instructions will get MQTT up and running on your **Ubuntu** system

### Installing project

[Download](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/emam2mqtt/-/archive/3-install-mqtt-linux/emam2mqtt-3-install-mqtt-linux.zip) this project to your computer

[Download](https://maven.apache.org/guides/getting-started/) and install Maven

[Download](https://www.eclipse.org/downloads/) and install Ecplise IDE

Import the project as "Maven Project" in Eclipse

Open terminal and switch to your project directory

In terminal run

`mvn clean install -s settings.xml`

### Installing MQTT (locally)

Open terminal

Add new MQTT repository

`sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa`

Run update

`sudo apt-get update`

Install mosquitto and mosquitto-clients

`sudo apt-get install mosquitto`

`sudo apt-get install mosquitto-clients`