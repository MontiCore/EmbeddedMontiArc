/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.Battery;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.InfiniteBattery;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.SimpleBattery;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotor;

public class ElectricalPowerTrain extends PowerTrain {
    public final Battery battery;
    public final ElectricMotor e_motor;    
    public final ElectricalPTProperties electricalPTProperties;

    private final double transmissionRatio;

    public ElectricalPowerTrain(ElectricalPTProperties properties) {
        super(properties);
        this.electricalPTProperties = properties;

        this.transmissionRatio = properties.transmissionRatio;

        switch (properties.batteryProperties.batteryType) {
            case INFINITE:
                battery = new InfiniteBattery();
                break;
            case SIMPLE:
                battery = new SimpleBattery();
                break;
            default:
            battery = null;
                break;
        }
        this.battery.init(properties.batteryProperties);

        switch(properties.motorProperties.type) {
            case DEFAULT:
                e_motor = new ElectricMotor();
                break;
            default:
                e_motor = null;
        }
        this.e_motor.setBattery(this.battery);
        this.e_motor.init(properties.motorProperties);
        this.motor = this.e_motor;
    }

    @Override
    public double getFuelPercentage() {
        return battery.percentage();
    }

    @Override
    public double getTransmissionRatio() {
        return transmissionRatio;
    }
}