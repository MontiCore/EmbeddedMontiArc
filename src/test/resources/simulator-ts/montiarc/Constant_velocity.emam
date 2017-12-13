package montiarc;

import montiarc.atomic.*;

<<Type = "SubSystem">> component Constant_velocity {
	port
		in (0km/h : 250km/h) velocity,
		in Double time,
		out (-2m/s^2:2m/s^2) acceleration,
		out (-180°:180°) steering;


	instance Constant<Double>(1) constant;
	instance Constant<Double>(0) constant1;
    instance Constant<Double>(2) constant2;
    instance Constant<Double>(0) constant3;
	instance Less<Double> relationalOperator;
	instance SwitchB<Double> switchBlock;

	connect constant3.out1 -> steering;
	connect switchBlock.out1 -> acceleration;
	connect relationalOperator.out1 -> switchBlock.cond;
	connect constant2.out1 -> switchBlock.in1;
	connect constant1.out1 -> switchBlock.in3;
	connect time -> relationalOperator.in1;
	connect constant.out1 -> relationalOperator.in2;
}
