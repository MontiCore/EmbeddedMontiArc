/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricMotor;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.InfiniteBattery;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.VehicleProperties;

public class DefaultVehicleSetup implements VehicleSetup {
    final ElectricalPTProperties powertrainProperties;
    final VehicleProperties properties;
    ElectricalPowerTrain powerTrain;

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
        return new RigidbodyPhysics(properties, powerTrain.motor, eesimulator, vehicle);
    }

    @Override
    public void addComponents(EESimulator eesimulator, Updater vehicle) {
        // TODO Auto-generated method stub
        // Speed sensor
        // Steering actuator/sensor -> PhysicsEngine
        // Gas pedal actuator/sensor -> PhysicsEngine
        // Braking actuator/sensor -> PhysicsEngine
        // Trajectory sensor
    }

}