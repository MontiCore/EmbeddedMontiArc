package visualization.doors;

import visualization.basics.Constant;
import visualization.basics.Equals;
import visualization.basics.Less;

component DoorStatus {
    port
        in (0km/h : 250km/h) velocity,
        out Boolean status;

    instance Constant<Double>(0.1) idle;
    instance Less<Double> less;

    //if the velocity of the car is less than idle than doorStatus is true
    connect velocity -> less.in1;
    connect idle.out1 -> less.in2;
    connect less.out1 -> status;
}