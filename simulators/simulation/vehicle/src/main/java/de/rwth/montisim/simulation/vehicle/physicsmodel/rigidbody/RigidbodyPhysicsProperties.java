/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

@Typed(RigidbodyPhysicsProperties.TYPE)
public class RigidbodyPhysicsProperties extends PhysicsProperties {
    public final static String TYPE = "rigidbody";

    @Override
    public PhysicsType getPhysicsType() {
        return PhysicsType.RIGIDBODY;
    }

    @Override
    public PhysicsModel build(PowerTrain powerTrain, VehicleProperties properties) {
        return new RigidbodyPhysics(this, powerTrain, properties);
    }

}