/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

@Typed(ElectricalPTProperties.TYPE)
public class ElectricalPTProperties extends PowerTrainProperties {
    public static final String TYPE = "electrical";

    public ElectricalPTProperties(ElectricMotorProperties motorProperties,
                                  BatteryProperties batteryProperties) {
        this.motorProperties = motorProperties;
        this.batteryProperties = batteryProperties;
    }

    public ElectricalPTProperties() {
        this.motorProperties = new ElectricMotorProperties();
        this.batteryProperties = new BatteryProperties();
    }


    @JsonEntry("motor")
    public ElectricMotorProperties motorProperties;
    @JsonEntry("battery")
    public BatteryProperties batteryProperties;

    public double transmission_ratio = 9; // 9:1

    @Override
    public PowerTrainType getPowerTrainType() {
        return PowerTrainType.ELECTRICAL;
    }

    @Override
    public PowerTrain build() {
        return new ElectricalPowerTrain(this);
    }


}