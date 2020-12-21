package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.commons.simulation.DynamicObject;

public class UpperSpeedLimit extends PhysicalValueDouble {
    public static final String VALUE_NAME = "upper_speed_limit";
    //transient final DynamicObject object;
    public UpperSpeedLimit(DynamicObject object) {
        super(VALUE_NAME);
    }

    /**
    * @return [x,y, upper_speed_limit] */
    @Override
    public Object get(){
        return 10;
    }

    @Override
    public void set(Object value){
        // Cannot change the velocity this way
    }
}
