/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor;

import de.rwth.montisim.simulation.vehicle.powertrain.Motor;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.Battery;

public class ElectricMotor implements Motor {
    transient Battery battery = null;
    // TODO
    public final transient ElectricMotorProperties properties;

    public ElectricMotor(ElectricMotorProperties properties) {
        this.properties = properties;
    }

    @Override
    public double getMaxTorque(double rpm) {
        // TODO Torque curve (following Back-EMF or oder model)
        if (battery.isCritical())
            return 0;
        return properties.motor_peek_torque;
    }

    @Override
    public void consume(double energy, double delta_t) {
        // TODO Auto-generated method stub
        this.battery.discharge(energy / properties.motor_efficiency);
    }

    @Override
    public void regenerate(double energy, double delta_t) {
        // TODO Auto-generated method stub
        this.battery.charge(energy * properties.regen_efficiency);
    }

    public void setBattery(Battery battery) {
        this.battery = battery;
    }

}