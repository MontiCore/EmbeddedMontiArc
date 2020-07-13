/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

@Typed("electrical")
public class ElectricalPTProperties extends PowerTrainProperties {
    public static final String TYPE = "electrical";

    public ElectricalPTProperties(VehicleProperties config, ElectricMotorProperties motorProperties,
            BatteryProperties batteryProperties) {
        super(PowerTrainType.ELECTRICAL, config);
        this.motorProperties = motorProperties;
        this.batteryProperties = batteryProperties;
    }

    public ElectricalPTProperties(VehicleProperties config) {
        super(PowerTrainType.ELECTRICAL, config);
        this.motorProperties = null;
        this.batteryProperties = null;
    }
    
    @JsonEntry("motor")
    public ElectricMotorProperties motorProperties;
    @JsonEntry("battery")
    public BatteryProperties batteryProperties;

    public double transmission_ratio = 9; // 9:1



    
}