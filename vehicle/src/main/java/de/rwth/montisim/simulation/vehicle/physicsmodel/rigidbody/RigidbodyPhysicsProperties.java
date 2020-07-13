/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;

@Typed("rigidbody")
public class RigidbodyPhysicsProperties extends PhysicsProperties {

    public RigidbodyPhysicsProperties() {
        super(PhysicsType.RIGIDBODY);
    }

}