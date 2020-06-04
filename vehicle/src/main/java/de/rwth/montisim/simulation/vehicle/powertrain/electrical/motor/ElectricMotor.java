/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor;

import de.rwth.montisim.simulation.vehicle.powertrain.Motor;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.Battery;

public class ElectricMotor implements Motor {
    Battery battery = null;
    //TODO
    public ElectricMotorProperties properties;

    public void init(ElectricMotorProperties properties){
        this.properties = properties;
    }

    @Override
    public double getMaxTorque(double rpm) {
        // TODO Torque curve (following Back-EMF or oder model)
        if (battery.charge < battery.criticalCharge)
            return 0;
        return properties.motorPeekTorque;
    }

    @Override
    public void consume(double energy, double delta_t) {
        // TODO Auto-generated method stub
        this.battery.discharge(energy/properties.motorEfficiency);
    }

    @Override
    public void regenerate(double energy, double delta_t) {
        // TODO Auto-generated method stub
        this.battery.charge(energy*properties.regenEfficiency);
    }

    public void setBattery(Battery battery){
        this.battery = battery;
    }
}