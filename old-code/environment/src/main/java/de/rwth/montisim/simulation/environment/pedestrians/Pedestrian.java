/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.pedestrians;

import de.rwth.montisim.commons.simulation.ISimulator;
import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.commons.simulation.StaticObject;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.simulation.environment.geometry.osmadapter.GeomStreet;
import de.rwth.montisim.simulation.environment.geometry.splines.StreetInterpolator;

import java.time.Duration;
import java.util.*;

/**
 * Class that represents a pedestrian in the simulation
 */
public class Pedestrian implements Updatable, SimulationObject {

    /** 4.5 km/h in meters per millisecond */
    private static final double PEDESTRIAN_SPEED_DEFAULT = 4.5 / 3600;

    StaticObject objectData;

    /** Variables for the IPedestrian interface */
    private PedestrianStreetParameters params;
    private GeomStreet geomStreet;
    private StreetInterpolator mInterpolator;

    /**
     * Constructor for a pedestrian that is standing at its position Initial
     * position at origin
     */
    public Pedestrian(GeomStreet geomStreet) {
        objectData = new StaticObject("Pedestrian");
        // TODO
        // width = 0.5;
        // length = 0.5;
        // height = 1.8;

        this.geomStreet = geomStreet;
        if (geomStreet != null) {
            mInterpolator = new StreetInterpolator(geomStreet);
        }
    }

    // TODO
    // /**
    // * Function that returns a list of pairs of 3D coordinates, indicating a
    // vector on the edges of the physical object
    // * @return List of pairs of 3D points, indicating a vector on the edges of the
    // physical object
    // */
    // @Override
    // @Deprecated
    // public List<Map.Entry<Vec3, Vec3>> getBoundaryVectors(){
    // //TODO: Function is unnecessary with three dimensional collision detection
    // // Build relative vectors between vertices
    // Vec3 relVectorBackFront = new Vec3(new double[] {0.0,
    // getLength(), 0.0});
    // Vec3 relVectorLeftRight = new Vec3(new double[] {getWidth(),
    // 0.0 , 0.0});
    // Vec3 relVectorBottomTop = new Vec3(new double[] {0.0, 0.0,
    // getHeight()});

    // // Rotate relative vectors
    // relVectorBackFront = getRotation().operate(relVectorBackFront);
    // relVectorLeftRight = getRotation().operate(relVectorLeftRight);
    // relVectorBottomTop = getRotation().operate(relVectorBottomTop);

    // // From center coordinate, compute to bottom left vertex of box
    // Vec3 absBackLeft = getGeometryPosition();
    // absBackLeft = absBackLeft.add(relVectorBackFront.mapMultiply(-0.5));
    // absBackLeft = absBackLeft.add(relVectorLeftRight.mapMultiply(-0.5));
    // absBackLeft = absBackLeft.add(relVectorBottomTop.mapMultiply(-0.5));

    // // Compute absolute vectors
    // Vec3 backLeft = absBackLeft.copy();
    // Vec3 backRight = absBackLeft.add(relVectorLeftRight);
    // Vec3 frontLeft = absBackLeft.add(relVectorBackFront);
    // Vec3 frontRight =
    // absBackLeft.add(relVectorLeftRight).add(relVectorBackFront);

    // // Put vectors in list and return
    // // Create map entries and insert them into list
    // // Ordering is important here
    // List<Map.Entry<Vec3, Vec3>> boundaryVectors = new LinkedList<>();
    // boundaryVectors.add(new AbstractMap.SimpleEntry<>(backLeft, backRight));
    // boundaryVectors.add(new AbstractMap.SimpleEntry<>(backRight, frontRight));
    // boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontRight, frontLeft));
    // boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontLeft, backLeft));
    // return boundaryVectors;
    // }

    // /**
    // * Function that sets the position of the center of mass and the rotation of
    // the object, in order to place the object on the surface of the world.
    // * given a x, y coordinate and a z rotation
    // * @param posX X component of the position of the physical object
    // * @param posY Y component of the position of the physical object
    // * @param rotZ Z component of the rotation of the physical object
    // */
    // @Override
    // public void putOnSurface(double posX, double posY, double rotZ){
    // double groundZ = WorldModel.getInstance().getGround(posX, posY,
    // this.getGeometryPosition().getEntry(2)).doubleValue();
    // this.setPosition(new Vec3(new double[] {posX, posY, groundZ + 0.5
    // * this.getHeight()}));
    // Rotation rot = new Rotation(RotationOrder.XYZ,
    // RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
    // this.setRotation(new BlockRealMatrix(rot.getMatrix()));
    // }

    public PedestrianStreetParameters getStreetParameters() {
        return params;
    }

    public void setStreetParameters(PedestrianStreetParameters newParams) {
        params = newParams;
        objectData.position = newParams.getPosition();
    }

    public void spawnAtRandomLocation(Random random) {
        PedestrianStreetParameters params = mInterpolator.spawnAtRandomLocation(random);
        setStreetParameters(params);
    }

    @Override
    public void update(Duration deltaT) {
        // get last movement parameters
        PedestrianStreetParameters movementParameters = this.getStreetParameters();

        /*
         * if (this.collision) { // If we have collided with another object we will stop
         * movement return; }
         */

        if (Math.random() < 0.0003 && !movementParameters.isCrossing()) {
            // So there is a 0.03% chances that we will cross the street
            // This is the case and we will be just updating the state with the crossing
            // flag
            movementParameters = new PedestrianStreetParameters(true, movementParameters.getPosition(),
                    movementParameters.isDirection(), movementParameters.isLeftPavement());
        }

        // compute new movement parameters
        // movement a pedestrians does in each time step
        double distance = PEDESTRIAN_SPEED_DEFAULT * deltaT.toMillis();
        // PedestrianStreetParameters newParams =
        // this.geomStreet.getMovementOfPedestrian(movementParameters, distance);
        PedestrianStreetParameters newParams = mInterpolator.calculateNewMovement(movementParameters, distance);

        // set new movement parameters in pedestrian
        this.setStreetParameters(newParams);

    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerStaticObject(objectData);
        simulator.registerUpdatable(this);
    }
}
