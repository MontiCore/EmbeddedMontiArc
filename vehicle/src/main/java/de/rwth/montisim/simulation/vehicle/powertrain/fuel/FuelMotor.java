/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

import de.rwth.montisim.simulation.vehicle.powertrain.Motor;

public class FuelMotor implements Motor {
    FuelTank tank = null;
    float litersPerJoule = 0; // TODO

    @Override
    public double getMaxTorque(double rpm) {
        // TODO Auto-generated method stub
        if (tank.capacity < 0.01) return 0;
        return 0;
    }

    @Override
    public void consume(double energy, double delta_t) {
        // TODO Auto-generated method stub
        tank.consume(energy*litersPerJoule);
    }

    @Override
    public void regenerate(double energy, double delta_t) {
        // Fuel based cars can't regenerate their energy.
    }

    public void setTank(FuelTank tank){
        this.tank = tank;
    }

}