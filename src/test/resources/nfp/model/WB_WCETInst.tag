/* (c) https://github.com/MontiCore/monticore */
package model;
conforms to nfp.TagLatencyTagSchema;
tags TagLatency for weatherBalloon.weatherBalloonSensors {
	tag gps1 with LatencyCmpInst = 950 ms;
	tag gps2 with LatencyCmpInst = 750 ms;
	tag temp1 with LatencyCmpInst = 500 ms;
	tag control with LatencyCmpInst = 50 ms ;
}
