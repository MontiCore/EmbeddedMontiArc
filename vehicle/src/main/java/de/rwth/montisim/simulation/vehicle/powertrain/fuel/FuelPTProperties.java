/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class FuelPTProperties extends PowerTrainProperties {
    public static final String TYPE = "fuel";

    public FuelPTProperties(VehicleProperties config) {
        super(PowerTrainType.FUEL_BASED, config);
    }

    double tank_capacity = 50; // In Liters
    double joules_per_liter = 32 * 1000000;


    public static final String K_TANK_CAPACITY = "tank_capacity";
    public static final String K_JOULES_PER_LITER = "joules_per_liter";

}