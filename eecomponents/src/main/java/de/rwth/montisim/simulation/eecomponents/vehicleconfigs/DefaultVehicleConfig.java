/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.vehicleconfigs;

import de.rwth.montisim.simulation.eecomponents.autopilots.JavaAutopilotProperties;
import de.rwth.montisim.simulation.eecomponents.lidar.LidarProperties;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitServiceProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.navigation.NavigationProperties;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties.BatteryType;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

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
                new SensorProperties()
                        .setPhysicalValueName(TrueVelocity.VALUE_NAME)
                        .setName("TrueVelocitySensor")
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                new SensorProperties()
                        .setPhysicalValueName(TruePosition.VALUE_NAME)
                        .setName("TruePositionSensor")
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                new SensorProperties()
                        .setPhysicalValueName(TrueCompass.VALUE_NAME)
                        .setName("TrueCompassSensor")
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                new SensorProperties()
                        .setPhysicalValueName(BatteryLevel.VALUE_NAME)
                        .setName("BatterySensor")
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                new NavigationProperties()
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                new LidarProperties()
                        .connectTo("DefaultBus")
        );

        properties.addComponent(
                new SpeedLimitServiceProperties()
                        .connectTo("DefaultBus")
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
                new JavaAutopilotProperties(maxForce / config.properties.body.mass)
                        .setName("TestAutopilot")
                        .connectTo("DefaultBus")
        );
        return config;
    }

    public DefaultVehicleConfig driveTo(double x, double y, double range) {
        properties.task.addGoal(new PathGoalProperties().reach(x, y).withinRange(range).eventually());
        return this;
    }

    public DefaultVehicleConfig setName(String vehicleName) {
        this.properties.vehicleName = vehicleName;
        return this;
    }

    public DefaultVehicleConfig setTask(TaskProperties task) {
        this.properties.task = task;
        return this;
    }

}