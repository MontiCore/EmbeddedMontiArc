/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.osmadapter;

import de.rwth.montisim.commons.simulation.StaticObject;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.pedestrians.PedestrianStreetParameters;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvObject;
import de.rwth.montisim.simulation.environment.visualisationadapter.Street;

// TODO BROKEN ?

/**
 * A container for geometric operations on Streets
 */
public class GeomStreet {

    private Street street;

    private SplineDeterminator deter;

    public GeomStreet(Street street) {
        this.street = street;
        this.deter = new LinearSplineDeterminator(street);
    }


    /**
     * @return the distance from node to the middle of the street
     */
    public double getDistanceToMiddle(StaticObject o) {
        //ensure EnvNode-Implementation implements equals() and hashCode()!!!!!
        EnvNode node = new EnvNode(o.pos.clone());

        if (street.getNodes().contains(node)) {
            return 0;
        } else {
            return deter.determineSplineDistance(o);
        }
    }

    /**
     * @return the distance from node to the middle of the street
     */
    public double getDistanceToMiddle(EnvNode node) {
        //ensure EnvNode-Implementation implements equals() and hashCode()!!!!!
        if (street.getNodes().contains(node)) {
            return 0;
        } else {
            return deter.determineSplineDistance(node);
        }
    }

    /**
     * @return the Distance to the end of the right lane of the street from this node.
     * Note: The lane is specified in the direction of travel! The right lane is the lane the car should drive on
     * if its course is correct. The left lane is normally the lane the car isn't driving on
     */
    public double getDistanceToRight(PhysicalObject o) {
        return this.deter.determineDistanceToRight(o);
    }

    /**
     * @return the Distance to the end of the left lane of the street from this node.
     * Note: The lane is specified in the direction of travel! The right lane is the lane the car should drive on
     * if its course is correct. The left lane is normally the lane the car isn't driving on
     */
    public double getDistanceToLeft(PhysicalObject o) {
        return this.deter.determineDistanceToLeft(o);
    }

    public double getDistancetoFrontLeft(PhysicalObject o) {
        return this.deter.determineDistanceFrontLeft(o);
    }

    public double getDistancetoFrontRight(PhysicalObject o) {
        return this.deter.determineDistanceFrontRight(o);
    }

    public double getGround(Vec3 pos) {
        return this.deter.getGround(pos);
    }


    /**
     * @return true iff node is on this street
     */
    public boolean contains(EnvNode node) {
        return this.deter.contains(node);
    }

    /**
     * @return the according EnvStreet-Object
     */
    public EnvObject getObject() {
        return this.street;
    }


    /**
     *
     * @param lastParameters
     * @param distance
     * @return the new Movement encapsulated as PedestrianStreetParameter
     */
    public PedestrianStreetParameters getMovementOfPedestrian(PedestrianStreetParameters lastParameters, double distance) {
        PedestrianStreetParameters newParams = this.deter.getMovementOfPedestrian(lastParameters, distance);

        // We need to check that we are still on the street to ensure we do not walk through the ground
        Vec3 calculatedPosition = newParams.getPosition();
        double locationZOnGround = this.deter.getGround(calculatedPosition);

        // Only update if we have a actual change
        if (locationZOnGround != calculatedPosition.z) {
            Vec3 positionOnGround = new Vec3(
                    calculatedPosition.asVec2(),
                    locationZOnGround
            );

            // Update with the correct z location
            newParams = new PedestrianStreetParameters(
                    newParams.isCrossing(),
                    positionOnGround,
                    newParams.isDirection(),
                    newParams.isLeftPavement()
            );
        }

        return newParams;
    }

    /**
     * initialises the first position and moving parameters of a pedestrian and spawns him onto a random spline
     * @return
     */
    public PedestrianStreetParameters spawnPedestrian() {
        return this.deter.spawnPedestrian();
    }

    /**
     * @param rightLane
     * @param p
     * @return nearest point on the corresponding lane to p
     */
    public Vec3 spawnCar(boolean rightLane, Vec3 p) {
        return this.deter.spawnCar(rightLane, p);
    }

    public SplineDeterminator getDeterminator() {
        return this.deter;
    }
}
