/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.Vec2;

public class TruePosition extends PhysicalValue {
    public static final String VALUE_NAME = "true_position";
    public static final DataType TYPE = DataType.VEC2;
    transient final DynamicObject object;
    public TruePosition(DynamicObject object) {
        super(VALUE_NAME, TYPE, new Vec2(0,0));
        this.object = object;
    }
    
    @Override
    public Object get(){
        return new Vec2(object.pos.x, object.pos.y);
    }
    @Override
    public void set(Object value){
        // Cannot change the position this way
    }
}