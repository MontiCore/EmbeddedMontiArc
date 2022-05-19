/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.boundingbox;

public interface BoundingBox {

    // These methods are intended to be used when the type of the bounding boxes is not known
    // If it is, one can save a virtual function call by directly use the functions in 'CollisionTests'
    // If one type is known, prefer using 'unknownType.collidesWith(knownType)'
    boolean collidesWith(BoundingBox other);

    boolean collidesWith(AABB other);

    boolean collidesWith(OBB other);

    boolean collidesWith(ConvexHull other);

    boolean collidesWith(Union other);
}