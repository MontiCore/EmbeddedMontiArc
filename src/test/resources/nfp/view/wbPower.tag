/* (c) https://github.com/MontiCore/monticore */
package view;
conforms to nfp.LatencyTagSchema;

tags Latency for wbView.Power1.WeatherBalloonSensors {
	tag  controlSignalsIn -> dataSaveInternalOut with ViewEffPower = 1 W;

}
