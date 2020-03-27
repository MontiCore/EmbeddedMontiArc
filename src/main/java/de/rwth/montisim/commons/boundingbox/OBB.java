/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.boundingbox;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec3;

public class OBB implements BoundingBox {
    public Vec3 position;
    public Vec3 halfSize;
    //Every collumn is a unit axis in the rotated space
    public Mat3 rotation;

    /// Takes ownership of the position and rotation
    public OBB(Vec3 position, Vec3 size, Mat3 rotation) {
        this.halfSize = size.multiply(0.5);
        this.position = position;
        this.rotation = rotation;
    }
}