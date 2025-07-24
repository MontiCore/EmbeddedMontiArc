/* (c) https://github.com/MontiCore/monticore */
package model;
conforms to nfp.CompPower;
tags CompPower for weatherBalloon {
	tag WeatherBalloonSensors with CompPower = 5 W;
	tag WeatherBalloonSensors.Temperature with CompPower = 400 mW;
	tag WeatherBalloonSensors.GPS with CompPower = 2500 mW;

}
