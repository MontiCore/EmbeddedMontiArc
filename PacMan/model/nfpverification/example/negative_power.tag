/* (c) https://github.com/MontiCore/monticore */
package example;

conforms to nfp.TagPowerTagSchema;

tags TagPower for negative {

    tag sensors with PowerCmpInst= 2 W;
	tag sensors.s_control1 with PowerCmpInst = 2 W;
	tag sensors.s_gps1 with PowerCmpInst = 50 mW;
	tag sensors.s_gps2 with PowerCmpInst = 50 mW;
	tag sensors.s_temp1 with PowerCmpInst = 50 mW;
	tag sensors.s_humid1 with PowerCmpInst = 50 mW;

    tag Sensors with PowerCmp = 1 W;
    tag units.Controller with PowerCmp = 50 mW;
    tag units.GPS with PowerCmp = 1 W;
    tag units.Temperature with PowerCmp = 100 mW;
    tag units.Humidity with PowerCmp = 100 mW;

}
