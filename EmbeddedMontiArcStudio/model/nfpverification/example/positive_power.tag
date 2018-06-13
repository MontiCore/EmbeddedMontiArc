package example;

conforms to nfp.TagPowerTagSchema;

tags TagPower for positive {

    tag sensors with PowerCmpInst= 300 mW;
	tag sensors.s_control1 with PowerCmpInst = 100 mW;
	tag sensors.s_gps1 with PowerCmpInst = 50 mW;
	tag sensors.s_gps2 with PowerCmpInst = 50 mW;
	tag sensors.s_temp1 with PowerCmpInst = 50 mW;
	tag sensors.s_humid1 with PowerCmpInst = 50 mW;

    tag Sensors with PowerCmp = 1 W;
    tag units.Controller with PowerCmp = 250 mW;
    tag units.GPS with PowerCmp = 250 mW;
    tag units.Temperature with PowerCmp = 250 mW;
    tag units.Humidity with PowerCmp = 250 mW;

}