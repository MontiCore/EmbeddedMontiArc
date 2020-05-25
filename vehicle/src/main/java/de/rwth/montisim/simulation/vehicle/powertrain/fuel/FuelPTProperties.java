/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.config.EEConfig;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class FuelPTProperties extends PowerTrainProperties {
    public FuelPTProperties(EEConfig eeConfig) {
        super(PowerTrainType.FUEL_BASED, eeConfig);
    }

    double tankCapacity = 50; // In Liters
    double joulesPerLiter = 32 * 1000000;
}