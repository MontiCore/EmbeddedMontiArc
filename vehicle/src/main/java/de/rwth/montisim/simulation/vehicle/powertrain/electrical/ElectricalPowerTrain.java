/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.simulation.vehicle.physicalvalues.BatteryLevel;
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

        battery = properties.batteryProperties.build();
        e_motor = properties.motorProperties.build();

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

    @Override
    public void registerPhysicalValues(PhysicalValueRegistry physicalValues) {
        super.registerPhysicalValues(physicalValues);
        physicalValues.addPhysicalValue(new BatteryLevel(battery));
    }

}