/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.object;

import de.rwth.montisim.commons.simulation.ISimulator;
import de.rwth.montisim.commons.simulation.IdGenerator;
import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.commons.simulation.StaticObject;
import de.rwth.montisim.commons.simulation.Updatable;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.*;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.util.Log;
import de.rwth.montisim.simulation.util.MathHelper;

import java.time.Duration;
import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Class that represents a house in the simulation
 */
public class House implements SimulationObject {

    private StaticObject objectData;

    public House() {
        objectData = new StaticObject("House");
        // TODO
        // objectData.bbox =
        // width = 1.0;
        // length = 1.0;
        // height = 0.5;
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerStaticObject(objectData);
    }


    // TODO
    // /**
    //  * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
    //  * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
    //  */
    // @Override
    // @Deprecated
    // public List<Map.Entry<Vec3, Vec3>> getBoundaryVectors(){
    //     // Build relative vectors between vertices
    //     Vec3 relVectorBackFront = new Vec3(new double[] {0.0, getLength(), 0.0});
    //     Vec3 relVectorLeftRight = new Vec3(new double[] {getWidth(), 0.0 , 0.0});
    //     Vec3 relVectorBottomTop = new Vec3(new double[] {0.0, 0.0, getHeight()});

    //     // Rotate relative vectors
    //     relVectorBackFront = getRotation().operate(relVectorBackFront);
    //     relVectorLeftRight = getRotation().operate(relVectorLeftRight);
    //     relVectorBottomTop = getRotation().operate(relVectorBottomTop);

    //     // From center coordinate, compute to bottom left vertex of box
    //     Vec3 absBackLeft = getGeometryPosition();
    //     absBackLeft = absBackLeft.add(relVectorBackFront.mapMultiply(-0.5));
    //     absBackLeft = absBackLeft.add(relVectorLeftRight.mapMultiply(-0.5));
    //     absBackLeft = absBackLeft.add(relVectorBottomTop.mapMultiply(-0.5));

    //     // Compute absolute vectors
    //     Vec3 backLeft = absBackLeft.copy();
    //     Vec3 backRight = absBackLeft.add(relVectorLeftRight);
    //     Vec3 frontLeft = absBackLeft.add(relVectorBackFront);
    //     Vec3 frontRight = absBackLeft.add(relVectorLeftRight).add(relVectorBackFront);

    //     // Put vectors in list and return
    //     // Create map entries and insert them into list
    //     // Ordering is important here
    //     List<Map.Entry<Vec3, Vec3>> boundaryVectors = new LinkedList<>();
    //     boundaryVectors.add(new AbstractMap.SimpleEntry<>(backLeft, backRight));
    //     boundaryVectors.add(new AbstractMap.SimpleEntry<>(backRight, frontRight));
    //     boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontRight, frontLeft));
    //     boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontLeft, backLeft));
    //     return boundaryVectors;
    // }


    // /**
    //  * Function that sets the position of the center of mass and the rotation of the object, in order to place the object on the surface of the world.
    //  * given a x, y coordinate and a z rotation
    //  * @param posX X component of the position of the physical object
    //  * @param posY Y component of the position of the physical object
    //  * @param rotZ Z component of the rotation of the physical object
    //  */
    // @Override
    // public void putOnSurface(double posX, double posY, double rotZ){
    //     double groundZ = WorldModel.getInstance().getGround(posX, posY, this.getGeometryPosition().getEntry(2)).doubleValue();
    //     this.setPosition(new Vec3(new double[] {posX, posY, groundZ + 0.5 * this.getHeight()}));
    //     Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
    //     this.setRotation(new BlockRealMatrix(rot.getMatrix()));
    // }

}
