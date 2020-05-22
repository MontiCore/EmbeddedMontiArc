/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.vehiclesetups;

import java.time.Duration;

import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.sensor.Sensor;
import de.rwth.montisim.simulation.vehicle.VehicleSetup;
import de.rwth.montisim.simulation.vehicle.autopilots.TestAutopilot;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricMotor;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.InfiniteBattery;
import de.rwth.montisim.simulation.vehicle.sensorvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.VehicleProperties;

public class DefaultVehicleSetup implements VehicleSetup {
    final ElectricalPTProperties powertrainProperties;
    final VehicleProperties properties;
    ElectricalPowerTrain powerTrain;
    RigidbodyPhysics physicsModel;

    public DefaultVehicleSetup() {
        this.powertrainProperties = new ElectricalPTProperties();
        this.properties = new VehicleProperties(powertrainProperties);
    }

    @Override
    public VehicleProperties getProperties() {
        return properties;
    }

    @Override
    public PowerTrain getPowerTrain(EESimulator ee_vehicle) {
        try {
            this.powerTrain = new ElectricalPowerTrain(ee_vehicle, powertrainProperties, InfiniteBattery.class,
            ElectricMotor.class);
            return this.powerTrain;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
    @Override
    public PhysicsModel getPhysicsModel(EESimulator eesimulator, Updater vehicle){
        this.physicsModel = new RigidbodyPhysics(properties, powerTrain, eesimulator, vehicle);
        return this.physicsModel;
    }

    @Override
    public void addComponents(EESimulator eesimulator, Updater vehicle) {
        // TODO Auto-generated method stub
        // Speed sensor
        // Steering actuator/sensor -> PhysicsEngine
        // Gas pedal actuator/sensor -> PhysicsEngine
        // Braking actuator/sensor -> PhysicsEngine
        // Trajectory sensor

        Bus bus = ConstantBus.newInstantBus(eesimulator, "ConstantBus");


        Sensor trueVel = new Sensor(
            eesimulator, "TrueVelocitySensor", 1, 
            new TrueVelocity(physicsModel.getPhysicalObject()), 
            Duration.ofMillis(100), // Update rate
            Duration.ofMillis(10), // Read time
            false, vehicle
        );


        double maxForce = powertrainProperties.motorPeekTorque *powerTrain.getTransmissionRatio()*2/properties.wheels.wheelDiameter;
        
        TestAutopilot autopilot = TestAutopilot.newCircleAutopilot(eesimulator, Duration.ofMillis(1), maxForce/properties.body.mass, 80, 30);
        //TestAutopilot autopilot = TestAutopilot.newStartStopAutopilot(eesimulator, Duration.ofMillis(1), maxForce/properties.body.mass, 80);

        physicsModel.connectActuatorsToBus(bus);
        trueVel.connectToBus(bus);
        autopilot.connectToBus(bus);
    }

}