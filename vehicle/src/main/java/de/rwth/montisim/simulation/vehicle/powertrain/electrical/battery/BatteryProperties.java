/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

public class BatteryProperties {
    public static enum BatteryType {
        INFINITE,
        SIMPLE
    }

    public final BatteryType batteryType;
    public BatteryProperties(BatteryType batteryType) {
        this.batteryType = batteryType;
    }

    // Partly taken from https://en.wikipedia.org/wiki/Tesla_Model_3
    
    public double batteryCapacity = 180 * 1000000; // In Joule
    public double batteryCriticalChargePercent = 10;
}