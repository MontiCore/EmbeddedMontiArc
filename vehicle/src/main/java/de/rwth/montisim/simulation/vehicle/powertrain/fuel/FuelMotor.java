/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.powertrain.Motor;

public class FuelMotor implements Motor {
    transient FuelTank tank = null;
    transient double litersPerJoule;

    public FuelMotor(FuelPTProperties properties) {
        this.litersPerJoule = 1.0 / properties.joules_per_liter;
    }

    @Override
    public double getMaxTorque(double rpm) {
        // TODO Auto-generated method stub
        if (tank.contained < 0.01) return 0;
        return 0;
    }

    @Override
    public void consume(double energy, double delta_t) {
        // TODO Auto-generated method stub
        tank.consume(energy * litersPerJoule);
    }

    @Override
    public void regenerate(double energy, double delta_t) {
        // Fuel based cars can't regenerate their energy.
    }

    public void setTank(FuelTank tank) {
        this.tank = tank;
    }

}