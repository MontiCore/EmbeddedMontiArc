/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.osmadapter;

import de.rwth.montisim.commons.simulation.StaticObject;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.geometry.splines.LinearInterpolator;
import de.rwth.montisim.simulation.environment.geometry.splines.Spline;
import de.rwth.montisim.simulation.environment.pedestrians.PedestrianStreetParameters;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvBounds;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.Street;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.Set;

/**
 * Created by lukas on 22.01.17.
 *
 * A street of n Nodes consists of n - 1 splines. This class is used to get the right spline for a given operation
 */
public abstract class SplineDeterminator {

    /**
     * A unique key for a spline
     */
    protected class Key {
        Vec3 p1;
        Vec3 p2;

        protected Key(Vec3 p1, Vec3 p2) {
            this.p1 = p1;
            this.p2 = p2;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            Key key = (Key) o;

            if (!p1.equals(key.p1)) return false;
            return p2.equals(key.p2);

        }

        @Override
        public int hashCode() {
            int result = p1.hashCode();
            result = 31 * result + p2.hashCode();
            return result;
        }
    }

    protected Street street;

    protected HashMap<Key, Spline> splines;

    protected EnvBounds bounds;

    public SplineDeterminator(Street street) {
        this.street = street;
        this.splines = new HashMap<>();
    }

    protected abstract void initBounds();

    /**
     * @param o
     * @return the distance to the middle spline
     */
    public double determineSplineDistance(StaticObject o) {
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), o.pos);
        return s.computeDistanceToMiddle(p);
    }

    /**
     * @param n
     * @return the distance to the middle spline
     */
    public double determineSplineDistance(EnvNode n) {
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), n.point);
        return s.computeDistanceToMiddle(n.point);
    }


    /**
     * @param o
     * @return the distance to the left spline
     */
    public double determineDistanceToLeft(StaticObject o) {
        Vec3 p = new Vec3(o.getGeometryPosition().getEntry(0), o.getGeometryPosition().getEntry(1), o.getGeometryPosition().getEntry(2));
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), p);
        return s.computeDistanceToLeft(o);
    }

    public double determineDistanceFrontLeft(StaticObject o) {
        Vec3 p = new Vec3(o.getGeometryPosition().getEntry(0), o.getGeometryPosition().getEntry(1), o.getGeometryPosition().getEntry(2));
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), p);
        return s.computeDistanceToFrontLeft(o);
    }

    public double determineDistanceFrontRight(StaticObject o) {
        Vec3 p = new Vec3(o.getGeometryPosition().getEntry(0), o.getGeometryPosition().getEntry(1), o.getGeometryPosition().getEntry(2));
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), p);
        return s.computeDistanceToFrontRight(o);
    }

    /**
     * @param o
     * @return the distance to the right spline
     */
    public double determineDistanceToRight(StaticObject o) {
        Vec3 p = new Vec3(o.getGeometryPosition().getEntry(0), o.getGeometryPosition().getEntry(1), o.getGeometryPosition().getEntry(2));
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), p);
        return s.computeDistanceToRight(o);
    }

    /**
     * @param n
     * @return true iff n is on the street or pavement of the street
     */
    public boolean contains(EnvNode n) {
        if (n.point.x > bounds.max.x) return false;
        if (n.point.y > bounds.max.y) return false;
        if (n.point.z > bounds.max.z) return false;
        if (n.point.x < bounds.min.x) return false;
        if (n.point.y < bounds.min.y) return false;
        if (n.point.z < bounds.min.z) return false;

        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), n.point);
        return s.isOnStreet(n.point) || s.isOnPavement(n.point);
    }

    /**
     * @param x
     * @param y
     * @param lastKnownZ
     * @return the corresponding z-Coordinate for given x, y and the last known z-Coordinate
     */
    public double getGround(Vec3 pos) {
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), pos);
        return s.getFloorForPoint(pos);
    }

    /**
     *
     * @param keys
     * @param p
     * @return the element of keys with Minimum Distance to p
     */

    private Spline getMinimumSplineForSetAndPoints(Set<Key> keys, Vec3 p) {
        double minDist = Double.MAX_VALUE;
        Spline result = null;

        if (keys.size() > 0) {
            result = this.splines.get(keys.toArray()[0]);
        }

        for (Key k : keys) {
            double dist = this.splines.get(k).computeDistanceToMiddle(p);
            if (dist < minDist) {
                minDist = dist;
                result = this.splines.get(k);
            }
        }
        return result;
    }


    /**
     * @return corresponding EnvStreet
     */
    public Street getStreet() {
        return this.street;
    }

    /**
     *
     * @param lastParameters
     * @param distance
     * @return the new Movement encapsulated as PedestrianStreetParameter
     */
    public PedestrianStreetParameters getMovementOfPedestrian(PedestrianStreetParameters lastParameters, double distance) {
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), lastParameters.getPosition());
        return s.computePointForPedestrian(lastParameters, distance);
    }

    /**
     * initialises the first position and moving parameters of a pedestrian and spawns him onto a random spline
     * @return
     */
    public PedestrianStreetParameters spawnPedestrian() {
        Random r = new Random();
        int splineIndex = r.nextInt(this.splines.keySet().size());
        Spline s = new ArrayList<Spline>(this.splines.values()).get(splineIndex);
        double t = r.nextDouble();
        boolean isCrossing = r.nextBoolean();
        boolean direction = r.nextBoolean();
        boolean leftPavement = r.nextBoolean();

        LinearInterpolator pavement = s.getPavement(leftPavement);
        Vec3 position = pavement.computePoint(t);

        return new PedestrianStreetParameters(isCrossing, position, direction, leftPavement);

    }

    public Vec3 spawnCar(boolean rightLane, Vec3 p) {
        Spline s = getMinimumSplineForSetAndPoints(this.splines.keySet(), p);
        return s.spawnCar(rightLane, p);
    }

    public Spline getSplineForPoints(Vec3 p1, Vec3 p2) {
        return this.splines.get(new Key(p1, p2));
    }
}
