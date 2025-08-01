/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public class FuelPowerTrain extends PowerTrain {
    transient public final FuelMotor f_motor;
    public final FuelTank tank;
    transient public final FuelPTProperties fuelPTProperties;

    public FuelPowerTrain(FuelPTProperties properties) {
        super(properties);
        this.fuelPTProperties = properties;
        this.f_motor = new FuelMotor(properties);
        this.motor = f_motor;
        this.tank = new FuelTank(properties);
        this.f_motor.setTank(tank);
    }

    @Override
    public double getFuelPercentage() {
        return tank.percentage();
    }

    @Override
    public double getTransmissionRatio() {
        // TODO Auto-generated method stub
        return 1;
    }

}