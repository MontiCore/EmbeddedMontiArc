package simulation.vehicle;

import commons.simulation.PhysicalObject;
import commons.simulation.PhysicalObjectType;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.util.MathHelper;
import java.util.List;
import java.util.Map;

/**
 * Physics calculations for simulation
 */
public class PhysicsEngine{
    /** Average earth gravitational acceleration */
    public static final double GRAVITY_EARTH = -9.81;
    /** Average air density */
    public static final double AIR_DENSITY = 1.225;
    /** Average road friction coefficient for dry roads (no unit) */
    public static final double ROAD_FRICTION_DRY = 0.7;
    /** Average road friction coefficient for wet roads (no unit) */
    public static final double ROAD_FRICTION_WET = 0.4;

    private PhysicsEngine(){
        // Private constructor to hide the implicit public one
    }

    /**
     * Computes the physics of a physical object
     *
     * @param object Is the physical object, which physics have to be computed
     * @param physicalObjects List of all physical objects, needed to check for collision
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    public static void computePhysics(PhysicalObject object, List<PhysicalObject> physicalObjects, long timeDiffMs){
        // Detect and handle possible collisions
        computeCollision(object, physicalObjects, timeDiffMs);

        object.computePhysics(timeDiffMs);
    }

    /**
     * Function that detects and handles a collision for a object
     *
     * @param object Is the physical object, which is the collision is computed for
     * @param physicalObjects List of all physical objects
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    private static void computeCollision(PhysicalObject object, List<PhysicalObject> physicalObjects, long timeDiffMs){
        //TODO: The collision computation currently detects and handles a collision between two cars twice
        // Do not compute collision if the object has a computational error
        if (object.getError()){
            return;
        }

        // Do not compute collision if the object is not a car
        if(object.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            return;
        }

        // Check objects for collisions
        for (PhysicalObject objectB : physicalObjects) {
            if(detectCollision(object, objectB)){
                object.setCollision(true);
                objectB.setCollision(true);
                calcCollisionForces(object, objectB, timeDiffMs);
            }
        }
    }

    /**
     * Function that detects a collision
     *
     * @param objectA First object tested for collision
     * @param objectB First object tested for collision
     * @return Weather a collision has occurred or not
     */
    private static boolean detectCollision(PhysicalObject objectA, PhysicalObject objectB){
        //TODO: Use better short cuts
        // Do not compute collision if its the same object
        if (objectA.getId() == objectB.getId()){
            return false;
        }

        // Do not compute if the other object has a computational error
        if (objectB.getError()){
            return false;
        }

        // Do not compute collision if both objects are more than 100 meters away from each other
        if (objectA.getGeometryPosition().getDistance(objectB.getGeometryPosition()) >= 100.0) {
            return false;
        }

        // Perform collision computation
        //TODO: Use a three dimensional collision detection
        List<Map.Entry<RealVector, RealVector>> boundariesA = objectA.getBoundaryVectors();
        List<Map.Entry<RealVector, RealVector>> boundariesB = objectB.getBoundaryVectors();
        return MathHelper.checkIntersection2D(boundariesA, boundariesB);
    }

    /**
     * Function that calculates forces caused by a collision
     *
     * @param objectA First object for which the force should be computed
     * @param objectB Second object for which force should be computed
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    private static void calcCollisionForces(PhysicalObject objectA, PhysicalObject objectB, long timeDiffMs){
        double deltaT = timeDiffMs / 1000.0;

        //Initial calculations for object A
        RealVector velocityA = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        double massA = 0.0;
        RealVector momentumA = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        if(objectA.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            PhysicalVehicle vehicleA = (PhysicalVehicle) objectA;
            velocityA = vehicleA.getVelocity();
            massA = vehicleA.getMass();
            momentumA = velocityA.mapMultiply(massA);
        }

        //Initial calculations for object B
        RealVector velocityB = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        double massB = 0.0;
        RealVector momentumB = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        if(objectB.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            PhysicalVehicle vehicleB = (PhysicalVehicle) objectB;
            velocityB = vehicleB.getVelocity();
            massB = vehicleB.getMass();
            momentumB = velocityB.mapMultiply(massB);
        }

        //Calculate new velocity
        RealVector velocityANew = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        RealVector velocityBNew = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        if(objectA.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            if(objectB.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
                //Both objects are cars
                RealVector temporaryCalculationA = velocityB;
                temporaryCalculationA = temporaryCalculationA.mapMultiply(2.0);
                temporaryCalculationA = temporaryCalculationA.subtract(velocityA);
                temporaryCalculationA = temporaryCalculationA.mapMultiply(massB);
                temporaryCalculationA = temporaryCalculationA.add(momentumA);
                velocityANew = temporaryCalculationA.mapDivide(massA + massB);
                RealVector temporaryCalculationB = velocityA;
                temporaryCalculationB = temporaryCalculationB.mapMultiply(2.0);
                temporaryCalculationB = temporaryCalculationB.subtract(velocityB);
                temporaryCalculationB = temporaryCalculationB.mapMultiply(massA);
                temporaryCalculationB = temporaryCalculationB.add(momentumB);
                velocityBNew = temporaryCalculationB.mapDivide(massB + massA);
            }else{
                //Only object A is a car
                velocityANew = velocityA.mapMultiply(-1.0);
            }
        }else{
            if(objectB.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
                //Only object B is a Car
                velocityBNew = velocityB.mapMultiply(-1.0);
            }else{
                //Both object are not a car
            }
        }

        //Calculate new Momentum and Force
        RealVector momentumANew = velocityANew.mapMultiply(massA);
        RealVector momentumBNew = velocityBNew.mapMultiply(massB);
        RealVector deltaMomentumA = momentumANew.subtract(momentumA);
        RealVector deltaMomentumB = momentumBNew.subtract(momentumB);
        RealVector forceA = deltaMomentumA.mapDivide(deltaT);
        RealVector forceB = deltaMomentumB.mapDivide(deltaT);

        //Add Force to the objects
        if(objectA.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            PhysicalVehicle vehicleA = (PhysicalVehicle) objectA;
            vehicleA.addForce(forceA);
        }
        if(objectB.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            PhysicalVehicle vehicleB = (PhysicalVehicle) objectB;
            vehicleB.addForce(forceB);
        }
    }
}