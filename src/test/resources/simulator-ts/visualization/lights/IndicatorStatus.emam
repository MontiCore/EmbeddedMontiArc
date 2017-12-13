package visualization.lights;

import visualization.basics.Constant;
import visualization.basics.Mod;
import visualization.basics.Less;

component IndicatorStatus {
    port
        in Double time,
        out Boolean status;

    instance Constant<Double>(2.0) divisor;
    instance Constant<Double>(1.0) condition;
    instance Mod<Double> mod;
    instance Less<Double> less;

    connect time -> mod.in1;
    connect divisor.out1 -> mod.in2;
    connect mod.out1 -> less.in1;
    connect condition.out1 -> less.in2;
    connect less.out1 -> status;
}