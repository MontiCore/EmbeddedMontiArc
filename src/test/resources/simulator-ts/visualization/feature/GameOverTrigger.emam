package visualization.feature;

import visualization.basics.Constant;
import visualization.basics.Greater;
import visualization.basics.Less;
import visualization.basics.Or;

component GameOverTrigger {
    port
        in N x,
        in N y,
        out Boolean status;

    // The boundaries of the 3D world
    instance Constant<Double>(-150) leftBoundary;
    instance Constant<Double>(150) rightBoundary;
    instance Constant<Double>(200) aboveBoundary;
    instance Constant<Double>(-200) belowBoundary;

    instance Greater<Double> greater1;
    instance Greater<Double> greater2;

    instance Less<Double> less1;
    instance Less<Double> less2;

    instance Or<Boolean> or1;
    instance Or<Boolean> or2;
    instance Or<Boolean> or3;

    // if the car drives out of the world than the Trigger Status is true
    connect x -> greater1.in1;
    connect aboveBoundary.out1 -> greater1.in2;
    connect x -> less1.in1;
    connect belowBoundary.out1 -> less1.in2;
    connect greater1.out1 -> or1.in1;
    connect less1.out1 -> or1.in2;

    connect y -> greater2.in1;
    connect rightBoundary.out1 -> greater2.in2;
    connect y -> less2.in1;
    connect leftBoundary.out1 -> less2.in2;
    connect greater2.out1 -> or2.in1;
    connect less2.out1 -> or2.in2;

    connect or1.out1 -> or3.in1;
    connect or2.out1 -> or3.in2;
    connect or3.out1 -> status;
}