/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.config;

import java.time.Duration;

import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties.BatteryType;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

public class TestVehicleConfig extends VehicleConfig {
    public final ElectricalPTProperties electricalPTProperties;
    public final RigidbodyPhysicsProperties rbPhysicsProperties;
    public final BatteryProperties batteryProperties;
    public final ElectricMotorProperties motorProperties;

    public TestVehicleConfig() {
        this.eeConfig = new EEConfig();
        this.properties = new VehicleProperties();
        this.batteryProperties = new BatteryProperties(BatteryType.INFINITE);
        this.motorProperties = new ElectricMotorProperties();
        this.electricalPTProperties = new ElectricalPTProperties(eeConfig, motorProperties, batteryProperties);
        this.powerTrainProperties = electricalPTProperties;
        this.rbPhysicsProperties = new RigidbodyPhysicsProperties();
        this.physicsProperties = rbPhysicsProperties;

        eeConfig.addComponent(
            new SensorProperties(
                Duration.ofMillis(100), // Update rate
                Duration.ofMillis(10), // Read time
                false
            )
            .setName("TrueVelocitySensor")
            .setPhysicalValueName(TrueVelocity.VALUE_NAME)
        );
        
        eeConfig.addComponent(
            ConstantBusProperties.instantBus().setName("DefaultBus")
        );
    }

}