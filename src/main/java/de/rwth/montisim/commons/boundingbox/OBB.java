/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.boundingbox;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec3;

public class OBB implements BoundingBox {
    public Vec3 offset = new Vec3(); // Offset from center of mass
    public Vec3 half_extent = new Vec3();
    public Mat3 axes = new Mat3();
}