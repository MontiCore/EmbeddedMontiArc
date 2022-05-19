/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.vehicleconfigs;

import java.time.Duration;

import de.rwth.montisim.simulation.eecomponents.autopilots.TestAutopilotProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties.BatteryType;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

public class TestVehicleConfig {
    public final VehicleProperties properties;

    public final ElectricalPTProperties electricalPTProperties;
    public final RigidbodyPhysicsProperties rbPhysicsProperties;
    public final BatteryProperties batteryProperties;
    public final ElectricMotorProperties motorProperties;

    public TestVehicleConfig() {
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
                        .setPhysicalValueName(TrueVelocity.VALUE_NAME)
                        .setName("TrueVelocitySensor")
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                ConstantBusProperties.instantBus().setName("DefaultBus")
        );


    }

    static public TestVehicleConfig newCircleAutopilotConfig(double maxSpeed, double turnRadius) {
        TestVehicleConfig config = new TestVehicleConfig();
        double maxForce = config.electricalPTProperties.motorProperties.motor_peek_torque *
                config.electricalPTProperties.transmission_ratio * 2 /
                config.properties.wheels.diameter;
        config.properties.addComponent(
                TestAutopilotProperties.circleAutopilot(Duration.ofMillis(1), maxForce / config.properties.body.mass, maxSpeed, turnRadius)
                        .setName("TestAutopilot")
                        .connectTo("DefaultBus")
        );
        return config;
    }

    static public TestVehicleConfig newStartStopAutopilotConfig(double targetSpeed) {
        TestVehicleConfig config = new TestVehicleConfig();
        double maxForce = config.electricalPTProperties.motorProperties.motor_peek_torque *
                config.electricalPTProperties.transmission_ratio * 2 /
                config.properties.wheels.diameter;
        config.properties.addComponent(
                TestAutopilotProperties.startStopAutopilot(Duration.ofMillis(1), maxForce / config.properties.body.mass, targetSpeed)
                        .setName("TestAutopilot")
                        .connectTo("DefaultBus")
        );
        return config;
    }

}