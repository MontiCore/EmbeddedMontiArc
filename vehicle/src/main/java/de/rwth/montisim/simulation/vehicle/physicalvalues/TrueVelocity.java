/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.IPM;

/**
 * Exposes the true "outside the simulation" velocity => Does not model imprecision due to measuring.
 * Converts the velocity to km/h
 */
public class TrueVelocity extends PhysicalValue {
    public static final String VALUE_NAME = "true_velocity";
    public static final DataType TYPE = DataType.DOUBLE;
    transient final DynamicObject object;
    public TrueVelocity(DynamicObject object) {
        super(VALUE_NAME, TYPE, 0);
        this.object = object;
    }
    
    @Override
    public Object get(){
        // Project the vehicle velocity on its front axis (+X)
        // the +X axis is incidentally the first column of the rotation matrix
        return IPM.dot(object.velocity, object.rotation.col1) * 3.6;
    }
    @Override
    public void set(Object value){
        // Cannot change the velocity this way
    }
}