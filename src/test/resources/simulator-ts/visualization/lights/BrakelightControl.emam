package visualization.lights;

import visualization.basics.Constant;
import visualization.basics.Greater;

component BrakelightControl {
    port
        in Double time,
        out Boolean status;

    instance Constant<Double>(1.0) timeout;
    instance Greater<Double> greater;

    // Turn brake lights on after 1 second
    connect time -> greater.in1;
    connect timeout.out1 -> greater.in2;
    connect greater.out1 -> status;
}