/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;

/**
 * Deviation in degrees from the X axis / West (positive and negative)
 */
public class TrueCompass extends PhysicalValue {
    public static final String VALUE_NAME = "true_compass";
    public static final DataType TYPE = DataType.DOUBLE;
    final DynamicObject object;
    final Vec2 v = new Vec2();
    public TrueCompass(DynamicObject object) {
        super(VALUE_NAME, TYPE, 0);
        this.object = object;
    }
    
    @Override
    public Object get(){
        // Project the vehicle orientation to the XY plane
        v.x = object.rotation.col1.x;
        v.y = object.rotation.col1.y;
        IPM.normalize(v);
        double angle = Math.acos(v.x) * Math.signum(v.y);
        return angle * Geometry.RAD_TO_DEG;
    }
    @Override
    public void set(Object value){
        // Cannot change the rotation this way
    }
}