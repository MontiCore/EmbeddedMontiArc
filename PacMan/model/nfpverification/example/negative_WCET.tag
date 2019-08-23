/* (c) https://github.com/MontiCore/monticore */
package example;

conforms to nfp.TagLatencyTagSchema;

tags TagLatency for negative {

    tag sensors with LatencyCmpInst = 1 s;
	tag sensors.s_control1 with LatencyCmpInst = 50 ms;
	tag sensors.s_gps1 with LatencyCmpInst = 950 ms;
	tag sensors.s_gps2 with LatencyCmpInst = 750 ms;
	tag sensors.s_temp1 with LatencyCmpInst = 55 ms;
	tag sensors.s_humid1 with LatencyCmpInst = 50 ms;

    tag Sensors with LatencyCmp = 5 s;
    tag Sensors.Controller with LatencyCmp = 50 ms;
    tag Sensors.GPS with LatencyCmp = 1 s;
    tag Sensors.Temperature with LatencyCmp = 100 ms;
    tag Sensors.Humidity with LatencyCmp = 100 ms;

}
