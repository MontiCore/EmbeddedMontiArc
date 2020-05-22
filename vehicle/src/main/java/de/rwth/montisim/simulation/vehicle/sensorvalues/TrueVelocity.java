package de.rwth.montisim.simulation.vehicle.sensorvalues;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.utils.IPM;

/**
 * Exposes the true "outside the simulation" velocity => Does not model imprecision due to measuring.
 * Converts the velocity to km/h
 */
public class TrueVelocity extends PhysicalValue {
    final DynamicObject object;
    public TrueVelocity(DynamicObject object) {
        super("velocity", 0);
        this.object = object;
    }
    
    @Override
    public double get(){
        // Project the vehicle velocity on its front axis (+X)
        // the +X axis is incidentally the first column of the rotation matrix
        return IPM.dot(object.velocity, object.rotation.col1) * 3.6;
    }
    @Override
    public void set(double value){
        // Cannot change the velocity this way
    }
}