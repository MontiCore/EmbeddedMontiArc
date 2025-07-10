/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.Battery;

/**
 * Current battery level in percent
 */
public class BatteryLevel extends PhysicalValueDouble {
    public static final String VALUE_NAME = "battery_level";
    transient final Battery battery;

    public BatteryLevel(Battery battery) {
        super(VALUE_NAME);
        this.battery = battery;
    }

    @Override
    public Object get() {
        return battery.percentage();
    }

    @Override
    public void set(Object value) {
        // cannot change the battery level this way
    }
}
