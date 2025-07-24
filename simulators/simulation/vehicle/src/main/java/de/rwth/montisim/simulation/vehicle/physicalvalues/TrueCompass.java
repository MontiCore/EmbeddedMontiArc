/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.commons.utils.*;

/**
 * Deviation in degrees from the X axis / West (positive and negative),
 * counter-clockwise
 */
public class TrueCompass extends PhysicalValueDouble {
    public static final String VALUE_NAME = "true_compass";
    transient final DynamicObject object;
    transient final Vec2 v = new Vec2();

    public TrueCompass(DynamicObject object) {
        super(VALUE_NAME);
        this.object = object;
    }

    @Override
    public Object get() {
        this.lastValue = this.value;
        // Project the vehicle orientation to the XY plane
        v.x = object.rotation.col1.x;
        v.y = object.rotation.col1.y;
        IPM.normalize(v);
        double angle = Math.acos(v.x) * Math.signum(v.y);
        this.value = angle * Geometry.RAD_TO_DEG;
        return Double.valueOf(this.value);
    }

    @Override
    public void set(Object value) {
        // Cannot change the rotation this way
    }
}