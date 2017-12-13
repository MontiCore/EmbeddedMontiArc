package visualization.movement;

import visualization.basics.Constant;
import visualization.basics.Greater;
import visualization.basics.GreaterEquals;
import visualization.basics.And;
import visualization.basics.SwitchB;
import visualization.basics.PlusMinus;
import visualization.basics.Less;

component SteeringControl {
    port
        // Distances of the sensors to the track
        in (0m:2m) fl, //frontleft
        in (0m:2m) fr, //frontright
        in (0m:2m) slf, //sideleftfront
        in (0m:2m) slb, //sideleftback
        in (0m:2m) srf, //siderightfront
        in (0m:2m) srb, //siderightback
        out (-180°:180°) steering;

    instance Constant<Double>(-5.0) steeringLeftValue; // if turn left than steer with -5°
    instance Constant<Double>(5.0) steeringRightValue; // if turn right than steer with +5°

    // if there is no proportion option of distances which tells us to turn left or right
    // than wiggle the car to calculate the distances new
    instance Constant<Double>(-1.0) wiggleValue;
    instance Constant<Double>(0.0) steeringNull;

    // A threshold which compares the distance and
    // just turn right/left if the difference between to distances is large enough
    instance Constant<Double>(0.1) threshold;
    instance Constant<Double>(-0.1) thresholdNeg;

    instance GreaterEquals<Double> greater1, greater2, greater3;
    instance And and1, and2, and3, and4, and5;
    instance SwitchB<Double> switchB1, switchB2, switchB3;
    instance PlusMinus<Double> minus1, minus2, minus3, minus4, minus5, minus6;
    instance Less<Double> less1, less2, less3, less4, less5;

    connect fl->minus1.in1;
    connect fr->minus1.in2;
    connect slf->minus2.in1;
    connect slb->minus2.in2;
    connect srb->minus3.in1;
    connect srf->minus3.in2;

    connect fl->minus4.in1;
    connect fr->minus4.in2;
    connect slf->minus5.in1;
    connect slb->minus5.in2;
    connect srb->minus6.in1;
    connect srf->minus6.in2;

    connect minus1.out1->greater1.in1;
    connect minus2.out1->greater2.in1;
    connect minus3.out1->greater3.in1;
    connect minus4.out1->less1.in1;
    connect minus5.out1->less2.in1;
    connect minus6.out1->less3.in1;

    connect threshold.out1->greater1.in2;
    connect threshold.out1->greater2.in2;
    connect threshold.out1->greater3.in2;
    connect thresholdNeg.out1->less1.in2;
    connect thresholdNeg.out1->less2.in2;
    connect thresholdNeg.out1->less3.in2;

    connect greater1.out1->and1.in1;
    connect greater2.out1->and1.in2;
    connect and1.out1->and3.in1;
    connect greater3.out1->and3.in2;

    connect less1.out1->and2.in1;
    connect less2.out1->and2.in2;
    connect and2.out1->and4.in1;
    connect less3.out1->and4.in2;

    connect and3.out1->switchB1.cond;
    connect steeringLeftValue.out1->switchB1.in1;

    connect and4.out1->switchB2.cond;
    connect steeringRightValue.out1->switchB2.in1;
    connect switchB2.out1->switchB1.in3;

    connect slf->less4.in1;
    connect slb->less4.in2;
    connect srf->less5.in1;
    connect srb->less5.in2;

    connect less4.out1->and5.in1;
    connect less5.out1->and5.in2;
    connect and5.out1->switchB3.cond;
    connect steeringNull.out1->switchB3.in3;
    connect wiggleValue.out1->switchB3.in1;
    connect switchB3.out1->switchB2.in3;

    connect switchB1.out1->steering;
}