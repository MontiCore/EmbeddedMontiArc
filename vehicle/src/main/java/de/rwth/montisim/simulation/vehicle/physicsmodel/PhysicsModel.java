/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.physicsmodel;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;

public interface PhysicsModel extends Updatable {

	DynamicObject getPhysicalObject();
    void setGroundPosition(Vec3 pos, Vec2 front);
}