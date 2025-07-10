/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.boundingbox;

import de.rwth.montisim.commons.utils.Vec3;

public class Collision {
    public Vec3 rel_pos = new Vec3();
    public Vec3 normal = new Vec3();
    public double penetration = 0;
}