/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.*;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotor;

public class ElectricalPowerTrain extends PowerTrain {
    public final Battery battery;
    transient public final ElectricMotor e_motor;
    transient public final ElectricalPTProperties electricalPTProperties;

    public ElectricalPowerTrain(ElectricalPTProperties properties) {
        super(properties);
        this.electricalPTProperties = properties;

        switch (properties.batteryProperties.type) {
            case INFINITE:
                battery = new InfiniteBattery(properties.batteryProperties);
                break;
            case SIMPLE:
                battery = new SimpleBattery(properties.batteryProperties);
                break;
            default:
                battery = null;
                break;
        }

        switch (properties.motorProperties.type) {
            case DEFAULT:
                e_motor = new ElectricMotor(properties.motorProperties);
                break;
            default:
                e_motor = null;
        }
        this.e_motor.setBattery(this.battery);
        this.motor = this.e_motor;
    }

    @Override
    public double getFuelPercentage() {
        return battery.percentage();
    }

    @Override
    public double getTransmissionRatio() {
        return electricalPTProperties.transmission_ratio;
    }

    
}