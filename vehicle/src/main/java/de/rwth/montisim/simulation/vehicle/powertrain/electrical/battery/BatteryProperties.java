/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import de.rwth.montisim.commons.utils.json.JsonEntry;

public class BatteryProperties {
    public static enum BatteryType {
        @JsonEntry("infinite")
        INFINITE,
        @JsonEntry("simple")
        SIMPLE
    }

    public BatteryType type;

    public BatteryProperties(BatteryType batteryType) {
        this.type = batteryType;
    }

    public BatteryProperties() {
        this.type = BatteryType.INFINITE;
    }

    // Partly taken from https://en.wikipedia.org/wiki/Tesla_Model_3

    public double capacity = 180 * 1000000; // In Joule
    public double critical_charge = 10; // In percent

    public Battery build() {
        switch (type) {
            case INFINITE:
                return new InfiniteBattery(this);
            case SIMPLE:
                return new SimpleBattery(this);
            default:
                throw new IllegalArgumentException("Missing Battery Type: " + type);
        }
    }

}