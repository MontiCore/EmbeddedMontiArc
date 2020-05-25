/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.simulation.vehicle.config.EEConfig;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

public class ElectricalPTProperties extends PowerTrainProperties {

    public ElectricalPTProperties(EEConfig eeConfig, ElectricMotorProperties motorProperties, BatteryProperties batteryProperties) {
        super(PowerTrainType.ELECTRICAL, eeConfig);
        this.motorProperties = motorProperties;
        this.batteryProperties = batteryProperties;
    }

    public ElectricMotorProperties motorProperties;
    public BatteryProperties batteryProperties;

    public double transmissionRatio = 9; // 9:1

}