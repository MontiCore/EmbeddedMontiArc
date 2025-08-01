/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.UMath;
import de.rwth.montisim.commons.utils.Vec2;

import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;

public class TruePosition extends PhysicalValue {
    public static final DataType TYPE = BasicType.VEC2;
    public static final String VALUE_NAME = "true_position";
    transient final DynamicObject object;
    Vec2 value;
    Vec2 lastValue;

    public TruePosition(DynamicObject object) {
        super(VALUE_NAME);
        this.object = object;
    }

    @Override
    public Object get() {
        lastValue = value;
        value = new Vec2(object.pos.x, object.pos.y);
        return value;
    }

    @Override
    public void set(Object value) {
        // Cannot change the position this way
    }

    @Override
    public DataType getType() {
        return BasicType.VEC2;
    }

    @Override
    public boolean hasChanged() {
        if (lastValue == null || value == null) return true;
        return !(
                UMath.equalsThreshold(lastValue.x, value.x, PhysicalValueDouble.EPSILON) &&
                        UMath.equalsThreshold(lastValue.y, value.y, PhysicalValueDouble.EPSILON)
        );
    }
}