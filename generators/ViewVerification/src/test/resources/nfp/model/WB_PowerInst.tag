/* (c) https://github.com/MontiCore/monticore */
package model;
conforms to nfp.CompPower;
tags CompPower for weatherBalloon.weatherBalloonSensors {
	tag gps1 with CompPowerInst = 2 W;
	tag gps2 with CompPowerInst = 2500 mW;
	tag temp1 with CompPowerInst = 400 mW ;
	tag control with CompPowerInst = 10 mW ;
}
