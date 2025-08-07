/* (c) https://github.com/MontiCore/monticore */
package view;
conforms to de.monticore.lang.embeddedmontiview.embeddedmontiview.tagging.LatencyTagSchema;

tags Latency for wbView.WCET1.WeatherBalloonSensors {
	tag controlSignalsIn -> dataSaveInternalOut with LatencyViewEff = 100 s;

}
