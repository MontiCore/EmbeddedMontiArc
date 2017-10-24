package view;
conforms to nfp.LatencyTagSchema;

tags Latency for wbView.WCET1.WeatherBalloonSensors {
	tag controlSignalsIn -> dataSaveInternalOut with LatencyViewEff = 100 s;
}