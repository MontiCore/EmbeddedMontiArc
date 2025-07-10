/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel;

import de.rwth.montisim.simulation.commons.DynamicObject;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;

public interface PhysicsModel extends Updatable {
    PhysicsProperties getProperties();

    DynamicObject getPhysicalObject();

    void setGroundPosition(Vec3 pos, Vec2 front);
}