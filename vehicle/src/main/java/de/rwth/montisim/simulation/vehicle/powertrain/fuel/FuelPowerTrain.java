/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public class FuelPowerTrain extends PowerTrain {
    FuelMotor f_motor;
    FuelTank tank;
    public FuelPowerTrain(FuelMotor f_motor, FuelTank tank) {
        super("FuelPowerTrain");
        this.motor = f_motor;
        this.f_motor = f_motor;
        this.tank = tank;
        this.f_motor.setTank(tank);
    }

    @Override
    public double getFuelPercentage() {
        return tank.percentage();
    }

}