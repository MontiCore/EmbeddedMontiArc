/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class FuelPTProperties extends PowerTrainProperties {
    public static final String TYPE = "fuel";

    double tank_capacity = 50; // In Liters
    double joules_per_liter = 32 * 1000000;

    @Override
    public PowerTrainType getPowerTrainType() {
        return PowerTrainType.FUEL_BASED;
    }

    @Override
    public PowerTrain build() {
        return new FuelPowerTrain(this);
    }

}