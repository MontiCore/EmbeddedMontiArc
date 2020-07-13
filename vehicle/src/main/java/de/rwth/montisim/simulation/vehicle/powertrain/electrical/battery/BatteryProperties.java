/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.json.Json;
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

    // Partly taken from https://en.wikipedia.org/wiki/Tesla_Model_3

    public double capacity = 180 * 1000000; // In Joule
    public double critical_charge = 10; // In percent

}