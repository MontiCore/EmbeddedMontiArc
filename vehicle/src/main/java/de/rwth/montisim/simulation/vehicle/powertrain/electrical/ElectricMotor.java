/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.simulation.vehicle.powertrain.Motor;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.ElectricalPTProperties;

public class ElectricMotor implements Motor {
    Battery battery = null;
    //TODO
    double motorEfficiency; // How much output energy per input electricity.
    double regenEfficiency;
    double peekTorque;

    public void init(ElectricalPTProperties properties){
        this.motorEfficiency = properties.motorEfficiency;
        this.regenEfficiency = properties.regenEfficiency;
        this.peekTorque = properties.motorPeekTorque;
    }

    @Override
    public double getMaxTorque(double rpm) {
        // TODO Torque curve (following Back-EMF or oder model)
        if (battery.charge < battery.criticalCharge)
            return 0;
        return peekTorque;
    }

    @Override
    public void consume(double energy, double delta_t) {
        // TODO Auto-generated method stub
        this.battery.discharge(energy/motorEfficiency);
    }

    @Override
    public void regenerate(double energy, double delta_t) {
        // TODO Auto-generated method stub
        this.battery.charge(energy*regenEfficiency);
    }

    public void setBattery(Battery battery){
        this.battery = battery;
    }
}