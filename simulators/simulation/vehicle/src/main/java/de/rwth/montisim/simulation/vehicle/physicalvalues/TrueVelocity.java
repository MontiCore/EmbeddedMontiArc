/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.commons.utils.IPM;

/**
 * Exposes the true "outside the simulation" velocity => Does not model imprecision due to measuring.
 * Converts the velocity to km/h
 */
public class TrueVelocity extends PhysicalValueDouble {
    public static final String VALUE_NAME = "true_velocity";
    transient final DynamicObject object;

    public TrueVelocity(DynamicObject object) {
        super(VALUE_NAME);
        this.object = object;
    }

    @Override
    public Object get() {
        // Project the vehicle velocity on its front axis (+X)
        // the +X axis is incidentally the first column of the rotation matrix
        this.lastValue = this.value;
        this.value = IPM.dot(object.velocity, object.rotation.col1) * 3.6;
        return Double.valueOf(this.value);
    }

    @Override
    public void set(Object value) {
        // Cannot change the velocity this way
    }
}