/* (c) https://github.com/MontiCore/monticore */
package model;
conforms to nfp.TagLatencyTagSchema;
tags TagLatency for weatherBalloon {
	tag WeatherBalloonSensors with LatencyCmp = 2 s;
	tag WeatherBalloonSensors.Temperature with LatencyCmp = 500 ms;
	tag WeatherBalloonSensors.GPS with LatencyCmp = 1 s;

}
