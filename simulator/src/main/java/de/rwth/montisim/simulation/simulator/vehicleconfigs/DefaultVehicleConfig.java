/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.vehicleconfigs;

import java.time.Duration;

import de.rwth.montisim.simulation.eecomponents.autopilots.JavaAutopilotProperties;
import de.rwth.montisim.simulation.eecomponents.navigation.NavigationProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties.BatteryType;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

public class DefaultVehicleConfig {
    public final VehicleProperties properties;

    public final ElectricalPTProperties electricalPTProperties;
    public final RigidbodyPhysicsProperties rbPhysicsProperties;
    public final BatteryProperties batteryProperties;
    public final ElectricMotorProperties motorProperties;

    public DefaultVehicleConfig() {
        this.properties = new VehicleProperties();
        this.batteryProperties = new BatteryProperties(BatteryType.INFINITE);
        this.motorProperties = new ElectricMotorProperties();
        this.electricalPTProperties = new ElectricalPTProperties(motorProperties, batteryProperties);
        this.properties.powertrain = electricalPTProperties;
        this.rbPhysicsProperties = new RigidbodyPhysicsProperties();
        this.properties.physics = rbPhysicsProperties;

        properties.addComponent(
            new SensorProperties(
                Duration.ofMillis(100), // Update rate
                Duration.ofMillis(10), // Read time
                false
            )
            .setName("TrueVelocitySensor")
            .setPhysicalValueName(TrueVelocity.VALUE_NAME)
        );

        properties.addComponent(
            new SensorProperties(
                Duration.ofMillis(100), // Update rate
                Duration.ofMillis(10), // Read time
                false
            )
            .setName("TruePositionSensor")
            .setPhysicalValueName(TruePosition.VALUE_NAME)
        );

        properties.addComponent(
            new SensorProperties(
                Duration.ofMillis(100), // Update rate
                Duration.ofMillis(10), // Read time
                false
            )
            .setName("TrueCompassSensor")
            .setPhysicalValueName(TrueCompass.VALUE_NAME)
        );

        properties.addComponent(
            new NavigationProperties().setName("Navigation")
        );
        
        properties.addComponent(
            ConstantBusProperties.instantBus().setName("DefaultBus")
        );
    }

    public static DefaultVehicleConfig withJavaAutopilot() {
        DefaultVehicleConfig config = new DefaultVehicleConfig();
        double maxForce = config.electricalPTProperties.motorProperties.motor_peek_torque *
            config.electricalPTProperties.transmission_ratio * 2 /
            config.properties.wheels.diameter;
            config.properties.addComponent(
                new JavaAutopilotProperties(maxForce/config.properties.body.mass).setName("TestAutopilot")
            );
        return config;
    }

    public DefaultVehicleConfig setName(String vehicleName){
        this.properties.vehicleName = vehicleName;
        return this;
    }

}