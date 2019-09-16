/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObject;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObjectType;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.WorldModel;
import simulation.environment.visualisationadapter.interfaces.EnvStreet.StreetPavements;
import simulation.util.MathHelper;
import simulation.util.OrientedBoundingBox;

import java.io.*;
import java.time.Duration;
import java.util.List;

/**
 * Physics calculations for simulation
 */
public class PhysicsEngine{
    /** Average earth gravitational acceleration */
    public static final double GRAVITY_EARTH = -9.81;
    /** Average air density */
    public static final double AIR_DENSITY = 1.225;

    private PhysicsEngine(){
        // Private constructor to hide the implicit public one
    }

    /**
     * Computes the physics of a physical object
     *
     * @param object Is the physical object, which physics have to be computed
     * @param physicalObjects List of all physical objects, needed to check for collision
     * @param timeDiff Difference in time
     */
    public static void computePhysics(PhysicalObject object, List<PhysicalObject> physicalObjects, Duration timeDiff){
        // Detect and handle possible collisions
        computeCollision(object, physicalObjects, timeDiff);

        object.computePhysics(timeDiff);
    }

    /**
     * Function that detects and handles a collision for a object
     *
     * @param object Is the physical object, which is the collision is computed for
     * @param physicalObjects List of all physical objects
     * @param timeDiff Difference in time
     */
    private static void computeCollision(PhysicalObject object, List<PhysicalObject> physicalObjects, Duration timeDiff){
        //TODO: The collision computation currently detects and handles a collision between two cars twice
        // Do not compute collision if the object has a computational error
        if (object.getError()){
            return;
        }

        // Do not compute collision if the object is not a car
        if(object.getPhysicalObjectType() != PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
            return;
        }

        // Check objects for collisions
        for (PhysicalObject objectB : physicalObjects) {
            if(detectCollision(object, objectB)){
                object.setCollision(true);
                objectB.setCollision(true);
                calcCollisionForces(object, objectB, timeDiff);
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

        // Construct bounding boxes
        OrientedBoundingBox boxA = new OrientedBoundingBox(objectA.getGeometryPosition(), objectA.getWidth(), objectA.getLength(), objectA.getHeight(), objectA.getRotation());
        OrientedBoundingBox boxB = new OrientedBoundingBox(objectB.getGeometryPosition(), objectB.getWidth(), objectB.getLength(), objectB.getHeight(), objectB.getRotation());

        // Perform collision computation and return result
        return MathHelper.checkIntersection(boxA, boxB);
    }

    /**
     * Function that calculates forces caused by a collision
     *
     * @param objectA First object for which the force should be computed
     * @param objectB Second object for which force should be computed
     * @param timeDiff Difference in time
     */
    private static void calcCollisionForces(PhysicalObject objectA, PhysicalObject objectB, Duration timeDiff){
        double deltaT = timeDiff.toMillis() / 1000.0;

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

    public static double calcFrictionCoefficient(RealVector v) {
        StreetPavements streetPavement = WorldModel.getInstance().getSurfaceType(v);
        boolean isItRaining = WorldModel.getInstance().isItRaining();

        return calcFrictionCoefficient(streetPavement, isItRaining);
    }

    public static double calcFrictionCoefficient(StreetPavements streetPavement, boolean isItRaining) {
        try {
            InputStream input = PhysicsEngine.class.getResourceAsStream("/FrictionCoefficient.csv");
            Reader in = new InputStreamReader(input);
            CSVParser csvParser = new CSVParser(in, CSVFormat.DEFAULT);

            switch(streetPavement) {
                case QUALITY: // Asphalt
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(2));
                    }

                case STONE:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(2));
                    }

                case PAVED:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(2));
                    }

                case DIRT:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(2));
                    }

                case UNPAVED:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(2));
                    }

                case GRASS:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(2));
                    }

                default:
                    return 1;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        return 1;
    }

    public static double calcRollingResistance(RealVector v, double pressure, double forceRoadFrictionBackFrontNorm) {
        double defaultValue = 0.005 + (1 / pressure) * (0.01 + 0.0095 * (forceRoadFrictionBackFrontNorm * 3.6 / 100) * (forceRoadFrictionBackFrontNorm * 3.6 / 100));
        try{
        StreetPavements streetPavement = WorldModel.getInstance().getSurfaceType(v);
        boolean isItRaining = WorldModel.getInstance().isItRaining();
        InputStream input = PhysicsEngine.class.getResourceAsStream("/FrictionCoefficient.csv");
        Reader in = new InputStreamReader(input);
        CSVParser csvParser = new CSVParser(in, CSVFormat.DEFAULT);


            switch (streetPavement) {
                case QUALITY:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(4));
                    }
                case STONE:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(4));
                    }
                case PAVED:// Asphalt
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(4));
                    }
                case DIRT:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(4));
                    }
                case UNPAVED:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(4));
                    }
                case GRASS:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(3)); // provisional value
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(4));
                    }

                default:
                    return defaultValue;
            }
        }catch (IOException e) {
            e.printStackTrace();
        }

        return defaultValue;
    }
}
