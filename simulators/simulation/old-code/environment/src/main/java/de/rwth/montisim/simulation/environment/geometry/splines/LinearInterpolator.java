/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.splines;

import de.rwth.montisim.commons.map.ControllerNode;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.pedestrians.PedestrianStreetParameters;

import java.util.ArrayList;
import java.util.Random;

/**
 * Created by lukas on 21.01.17.
 *
 * This class uses linear interpolation to interpolate a street
 * see http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html for details on the math
 */
public class LinearInterpolator implements Spline {
    protected static final long PAVEMENT_ID = -1L;

    //    protected static final double PAVEMENT_WIDTH = 2;
    protected static final double PAVEMENT_WIDTH = 1;

    //set Precision to 1 nm
    public static final double precision = Math.pow(10, -9);

    //an enum for directions
    protected enum Direction {
        LEFT, MIDDLE, RIGHT
    }

    /**
     * construct borders and pavements of a street by parallel movement
     */

    protected class PointContainer {
        Vec3 p;
        Vec3 n1;
        Vec3 n2;

        Vec3 pavement1;
        Vec3 pavement2;

        Vec3 normalVec;

        protected PointContainer(Vec3 p, Vec3 difference, double streetWidth, double pavementWidth) {
            this.p = p;
            initPointContainer(difference, streetWidth, pavementWidth);
        }

        //TODO: compute pavement start and endPoints correct!
        protected void initPointContainer(Vec3 difference, double streetWidth, double pavementWidth) {
            //normalize the difference vector

            //compute the normal-vector
            this.normalVec = new Vec3(-difference.y, difference.x, 0);
            this.normalVec = this.normalVec.normalize();
            //construct other points by parallel movement
            Vec3 n1 = this.p.add(normalVec.multiply(0.5 * streetWidth));
            Vec3 pavement1 = this.p.add(normalVec.multiply(0.5 * streetWidth + 0.5 * pavementWidth));
            Vec3 n2 = this.p.add(normalVec.multiply(-0.5 * streetWidth));

            Vec3 pavement2 = this.p.add(normalVec.multiply(-0.5 * streetWidth - 0.5 * pavementWidth));
            //ensure that n1 is always the left lane and n2 the right lane (pavements accordingly
            if (n1.x < p.x || (n1.x == p.x && n1.y > p.y)) {
                this.n1 = n1;
                this.pavement1 = pavement1;
                this.n2 = n2;
                this.pavement2 = pavement2;
            } else {
                this.n1 = n2;
                this.pavement1 = pavement2;
                this.n2 = n1;
                this.pavement2 = pavement1;
            }
        }
    }

    protected Vec3 difference;

    protected Vec3 difference2D;

    protected PointContainer p1In3d;
    protected PointContainer p2In3d;

    //some computations have to be made in 2D-Space
    protected PointContainer p1In2d;
    protected PointContainer p2In2d;

    protected double streetWidth;

    protected long osmId1;
    protected long osmId2;

    protected LinearInterpolator leftPavement;
    protected LinearInterpolator rightPavement;

    protected boolean withPavements;

    /**
     *
     * @param p1 first Point of the linear interpolation
     * @param p2 second Point of the linear interpolation
     * @param streetWidth width of the street
     * @param osmId1 osmId of the first point
     * @param osmId2 osmId of the second point
     * @param withPavements true if interpolator should generate pavements
     */
    public LinearInterpolator(Vec3 p1, Vec3 p2, double streetWidth, long osmId1, long osmId2, boolean withPavements) {
        init(p1, p2, streetWidth, osmId1, osmId2, withPavements, PAVEMENT_WIDTH);
        if (withPavements) {
            this.leftPavement = new LinearInterpolator(this.p1In3d.pavement1, this.p2In3d.pavement1, PAVEMENT_WIDTH, PAVEMENT_ID, PAVEMENT_ID, false);
            this.rightPavement = new LinearInterpolator(this.p1In3d.pavement2, this.p2In3d.pavement2, PAVEMENT_WIDTH, PAVEMENT_ID, PAVEMENT_ID, false);
        }
    }

    public LinearInterpolator(Vec3 p1, Vec3 p2, double streetWidth, long osmId1, long osmId2, boolean withPavements, double pavementWidth) {
        init(p1, p2, streetWidth, osmId1, osmId2, withPavements, pavementWidth);
        if (withPavements) {
            this.leftPavement = new LinearInterpolator(this.p1In3d.pavement1, this.p2In3d.pavement1, pavementWidth, PAVEMENT_ID, PAVEMENT_ID, false);
            this.rightPavement = new LinearInterpolator(this.p1In3d.pavement2, this.p2In3d.pavement2, pavementWidth, PAVEMENT_ID, PAVEMENT_ID, false);
        }
    }

    private void init(Vec3 p1, Vec3 p2, double streetWidth, long osmId1, long osmId2, boolean withPavements, double pavementWidth) {
        this.streetWidth = streetWidth;

        this.difference = computeDifference(p2, p1);
        this.p1In3d = new PointContainer(p1, difference, streetWidth, pavementWidth);
        this.p2In3d = new PointContainer(p2, difference, streetWidth, pavementWidth);

        this.difference2D = new Vec3(difference.x, difference.y, 0);
        this.p1In2d = new PointContainer(new Vec3(p1.x, p1.y, 0), difference2D, streetWidth, pavementWidth);
        this.p2In2d = new PointContainer(new Vec3(p2.x, p2.y, 0), difference2D, streetWidth, pavementWidth);

        this.osmId1 = osmId1;
        this.osmId2 = osmId2;
        this.withPavements = withPavements;
    }

    /**
     *
     * @param p2
     * @param p1
     * @return p2In3d - p1In3d
     */
    private static Vec3 computeDifference(Vec3 p2, Vec3 p1) {
        return p2.subtract(p1);
    }

    @Override
    public Vec3 computePoint(double t) {
        return p1In3d.p.add(difference.multiply(t));
    }

    @Override
    public double computeDistanceToMiddle(Vec3 p) {
        return computeDistance(p, Direction.MIDDLE, true);
    }

    @Override
    public double computeDistanceToLeft(Vec3 p) {
        return computeDistance(p, Direction.LEFT, true);
    }

    // @Override
    // public double computeDistanceToLeft(PhysicalObject o) {
    //     Vec3 left = o.getGeometryPosition().add(o.getRotation().operate(new Vec3(new double[] {-0.5 * o.getWidth(), 0.0, 0.0})));
    //     Vec3 right = o.getGeometryPosition().add(o.getRotation().operate(new Vec3(new double[] {0.5 * o.getWidth(), 0.0, 0.0})));
    //     Vec3 middle = o.getGeometryPosition();

    //     Vec3 lP = new Vec3(left.getEntry(0),left.getEntry(1),left.getEntry(2));
    //     Vec3 rP = new Vec3(right.getEntry(0),right.getEntry(1),right.getEntry(2));
    //     Vec3 mP = new Vec3(middle.getEntry(0),middle.getEntry(1),middle.getEntry(2));

    //     double lDist = computeDistance(lP, Direction.LEFT, Direction.RIGHT, false);
    //     double rDist = computeDistance(rP, Direction.LEFT, Direction.RIGHT, false);

    //     if(lDist < rDist) {
    //         return computeDistance(mP, Direction.LEFT, Direction.RIGHT, false);
    //     } else {
    //         return computeDistance(mP, Direction.LEFT, Direction.LEFT, false);
    //     }
    // }

    // @Override
    // public double computeDistanceToFrontLeft(PhysicalObject o){
    //     Vec3 front = o.getGeometryPosition().add(o.getRotation().operate(new Vec3(new double[]{-0.25*o.getWidth(),5*o.getLength(),0.0})));
    //     Vec3 frontpoint= new Vec3(front.getEntry(0),front.getEntry(1),front.getEntry(2));
    //     return computeDistance(frontpoint,Direction.MIDDLE,false);
    // }

    // @Override
    // public double computeDistanceToFrontRight(PhysicalObject o){
    //     Vec3 front = o.getGeometryPosition().add(o.getRotation().operate(new Vec3(new double[]{0.25*o.getWidth(),5*o.getLength(),0.0})));
    //     Vec3 frontpoint= new Vec3(front.getEntry(0),front.getEntry(1),front.getEntry(2));
    //     return computeDistance(frontpoint,Direction.MIDDLE,false);
    // }

    @Override
    public double computeDistanceToRight(Vec3 p) {
        return computeDistance(p, Direction.RIGHT, true);
    }

    // @Override
    // public double computeDistanceToRight(PhysicalObject o) {
    //     Vec3 left = o.getGeometryPosition().add(o.getRotation().operate(new Vec3(new double[] {-0.5 * o.getWidth(), 0.0, 0.0})));
    //     Vec3 right = o.getGeometryPosition().add(o.getRotation().operate(new Vec3(new double[] {0.5 * o.getWidth(), 0.0, 0.0})));
    //     Vec3 middle = o.getGeometryPosition();

    //     Vec3 lP = new Vec3(left.getEntry(0),left.getEntry(1),left.getEntry(2));
    //     Vec3 rP = new Vec3(right.getEntry(0),right.getEntry(1),right.getEntry(2));
    //     Vec3 mP = new Vec3(middle.getEntry(0),middle.getEntry(1),middle.getEntry(2));


    //     double lDist = computeDistance(lP, Direction.RIGHT, Direction.RIGHT, false);
    //     double rDist = computeDistance(rP, Direction.RIGHT, Direction.RIGHT, false);

    //     if(rDist < lDist) {
    //         return computeDistance(mP, Direction.RIGHT, Direction.RIGHT, false);
    //     } else {
    //         return computeDistance(mP, Direction.RIGHT, Direction.LEFT, false);
    //     }

    // }

    @Override
    public boolean isOnStreet(Vec3 p) {
        return Math.abs(streetWidth - (computeDistance(p, Direction.RIGHT, false) + computeDistance(p, Direction.LEFT, false))) <= precision
                && p.z == getFloorForPoint(p);
    }

    @Override
    public boolean isOnPavement(Vec3 p) {
        return leftPavement.isOnStreet(p) || rightPavement.isOnStreet(p);
    }

    /**
     * @param p
     * @param d
     * @param in3D
     * @param lane
     * @return the distance for a given point p and direction d
     */
    private double computeDistance(Vec3 p, Direction d, Direction lane, boolean in3D) {
        Vec3 s;

        if (d == Direction.LEFT) {
            if (lane == Direction.RIGHT) {
                s = in3D ? p1In3d.n1 : p1In2d.n1;
            } else {
                s = in3D ? p1In3d.n2 : p1In2d.n2;
            }

        } else if (d == Direction.MIDDLE) {
            s = in3D ? p1In3d.p : p1In2d.p;
        } else {
            if (lane == Direction.RIGHT) {
                s = in3D ? p1In3d.n2 : p1In2d.n2;
            } else {
                s = in3D ? p1In3d.n1 : p1In2d.n1;
            }
        }

        double squaredNormOfPointsOnLine = getSquaredNorm(difference);
        Vec3 differenceOfP1AndP = computeDifference(s, p);

        double numerator = getSquaredNorm(differenceOfP1AndP) * squaredNormOfPointsOnLine - Math.pow(differenceOfP1AndP.dotProduct(difference), 2);
        double denominator = squaredNormOfPointsOnLine;

        // Prevent invalid math operations here to avoid exceptions
        if (numerator < 0.0) {
            numerator = 0.0;
        }

        if (denominator <= 0.0) {
            denominator = 1E-10;
        }

        return Math.sqrt(numerator / denominator);
    }

    private double computeDistance(Vec3 p, Direction d, boolean in3D) {
        return computeDistance(p, d, determineLaneOfPoint(p), in3D);
    }

    /**
     * @param p
     * @return the squared euclidean norm of given point p
     */
    private static double getSquaredNorm(Vec3 p) {
        return p.dotProduct(p);
    }

    /**
     * @param p
     * @return returns t for the nearest point on the spline
     */
    public double computeT(Vec3 p) {
        return computeT(p, true);
    }

    public double computeT(Vec3 p, boolean in3D) {
        Vec3 tmpP;
        Vec3 tmpDifference;
        Vec3 tmpPC;

        if (in3D) {
            tmpP = p;
            tmpDifference = difference;
            tmpPC = p1In3d.p;
        } else {
            tmpP = new Vec3(p.x, p.y, 0);
            tmpDifference = difference2D;
            tmpPC = p1In2d.p;
        }
        Vec3 differenceOfp1AndP = computeDifference(tmpPC, tmpP);
        return -1 * (differenceOfp1AndP.dotProduct(tmpDifference) / getSquaredNorm(tmpDifference));
    }

    @Override
    public double getFloorForPoint(Vec3 p) {
        return computePoint(computeT(p, false)).z;
    }

    // @Override
    // @Deprecated
    // public ArrayList<IControllerNode> convertToControllerList() {
    //     ArrayList<IControllerNode> result = new ArrayList<>();
    //     Vec3 normDiff = difference.normalize().multiply(IControllerNode.INTERPOLATION_DISTANCE);

    //     result.add(new ControllerNode(p1In3d.p, osmId1));
    //     double distance = p1In3d.p.distance(p2In3d.p);


    //     double loops = distance / IControllerNode.INTERPOLATION_DISTANCE;
    //     loops = Math.floor(loops);

    //     Vec3 tmpP = p1In3d.p;
    //     for(int i = 0; i < loops; i++) {
    //         tmpP = tmpP.add(normDiff);
    //         result.add(new ControllerNode(tmpP, IControllerNode.INTERPOLATED_NODE));
    //     }

    //     result.add(new ControllerNode(p2In3d.p, osmId2));
    //     return result;
    // }

    @Override
    public Vec3 getDifference() {
        return this.difference;
    }

    @Override
    public Vec3 getP1() {
        return this.p1In3d.p;
    }

    @Override
    public Vec3 getP2() {
        return this.p2In3d.p;
    }

    @Override
    public Vec3 getBorder(boolean left, boolean isP1) {
        PointContainer tmp;
        if (isP1) {
            tmp = p1In3d;
        } else {
            tmp = p2In3d;
        }

        if (left) {
            return tmp.n1;
        } else {
            return tmp.n2;
        }
    }

    public ArrayList<Vec3> getAllBorders() {
        ArrayList<Vec3> result = new ArrayList<>();
        result.add(getBorder(false, false));
        result.add(getBorder(false, true));
        result.add(getBorder(true, false));
        result.add(getBorder(true, true));

        if (this.leftPavement != null) {
            result.addAll(leftPavement.getAllBorders());
        }

        if (this.rightPavement != null) {
            result.addAll(rightPavement.getAllBorders());
        }

        return result;
    }


    /**
     * @param p
     * @return the lane of a given Point
     */
    //TODO: check back with other groups
    private Direction determineLaneOfPoint(Vec3 p) {
        Vec3 p0 = computePoint(computeT(p));
        if (p0.x == p.x && p0.y == p.y) {
            return Direction.MIDDLE;
        }

        if (p.x < p0.x) {
            return Direction.LEFT;
        } else if (p.x == p0.x && p.y > p0.y) {
            return Direction.LEFT;
        } else {
            return Direction.RIGHT;
        }
    }

    /**
     *
     * @param lastResult
     * @param distance
     * @return a container which encapsulates the new movement of the pedestrian
     */
    @Override
    public PedestrianStreetParameters computePointForPedestrian(PedestrianStreetParameters lastResult, double distance) {
        Vec3 position = lastResult.getPosition();
        boolean isCrossing = lastResult.isCrossing();
        boolean isDirection = lastResult.isDirection();
        boolean leftPavement = lastResult.isLeftPavement();

        if (isCrossing) {
            return crossStreet(lastResult, distance);
        } else {
            return walkNormal(position, distance, isDirection, leftPavement);
        }

    }

    /**
     * @param lastResult
     * @param distance
     * @return the next point if the pedestrian crosses the street in dependence of his position and the pavement he is supposed to be walking on
     * Note: distance(position, next point) = distance
     */
    private PedestrianStreetParameters crossStreet(PedestrianStreetParameters lastResult, double distance) {
        Vec3 position = lastResult.getPosition();
        boolean isOnLeftPavement = lastResult.isLeftPavement();

        // Resulting position
        Vec3 result;// = position.add(p1In3d.normalVec.normalize().multiply(distance));

        LinearInterpolator targetPavement;
        Vec3 direction;

        if (isOnLeftPavement) {
            targetPavement = rightPavement;
            direction = computeDifference(rightPavement.p1In3d.p, p1In3d.p);
        } else {
            targetPavement = leftPavement;
            direction = computeDifference(leftPavement.p1In3d.p, p1In3d.p);
        }

        // Normalize so we can use it as a direction
        direction = direction.normalize();
        result = position.add(direction.multiply(distance));

        // distance to other pavement has to become smaller if the pedestrian crosses the street
        double resultingDistance = targetPavement.computeDistanceToMiddle(result);
        double previousDistance = targetPavement.computeDistanceToMiddle(position);

        if (resultingDistance > previousDistance) {
            // So the distance has increased

            if (!targetPavement.isOnStreet(result)) {
                // He were going beyond the pavement; we should go a little bit back
                result = result.subtract(p1In3d.normalVec.normalize().multiply(distance * 0.49));
            }

            // We are on the correct side of the street; NOTE: "isOnStreet" --> is on the pavement
            // the next street parameter should not be in the crossing state
            return new PedestrianStreetParameters(
                    false,
                    result,
                    new Random().nextBoolean(), // We do not really care in what direction we should now go
                    !isOnLeftPavement // We are no longer on the current side
            );
        }

        // we continue crossing the street
        return new PedestrianStreetParameters(
                true,
                result,
                false, // the direction won't matter since we neither walk left or right
                isOnLeftPavement // Until we reach the end we are on the same side
        );
    }

    /**
     * @param position
     * @param distance
     * @param isDirection
     * @param onLeftPavement
     * @return the new Position if the pedestrian continues to walk on his pavement.
     * Note: If a pedestrian reaches the end of the spline he turns around
     */
    private PedestrianStreetParameters walkNormal(Vec3 position, double distance, boolean isDirection, boolean onLeftPavement) {
        LinearInterpolator pavement;

        if (onLeftPavement) {
            pavement = leftPavement;
        } else {
            pavement = rightPavement;
        }

        boolean resultDir = isDirection;
        Vec3 result = pavement.getPointWithDistance(position, distance, isDirection);

        double lengthOfPavement = Math.sqrt(getSquaredNorm(pavement.getDifference()));

        // Test if we have gone too far ahead
        if (getDistanceFromInterpolationPoint(isDirection, result) >= lengthOfPavement) {
            // Too far ahead so turn around
            resultDir = !isDirection;

            // Move in the opposite direction
            result = pavement.getPointWithDistance(position, distance, resultDir);

            // Test if even this is too far ahead
            if (getDistanceFromInterpolationPoint(resultDir, result) >= lengthOfPavement) {
                // We are even now too far ahead, so go ahead and stay where we are
                return new PedestrianStreetParameters(false, position, isDirection, onLeftPavement);
            }
        }

        return new PedestrianStreetParameters(false, result, resultDir, onLeftPavement);
    }

    /**
     * @param p1
     * @param distance
     * @param isDirection
     * @return a point p such that distance(p1In3d, p) = distance holds and in dependence of a direction
     */
    public Vec3 getPointWithDistance(Vec3 p1, double distance, boolean isDirection) {
        Vec3 movement = this.difference.normalize().multiply(distance);
        if (isDirection) {
            return p1.add(movement);
        } else {
            return p1.subtract(movement);
        }
    }

    /**
     * @param isP1
     * @param p
     * @return the distance of p to p1In3d (or p2In3d)
     */
    public double getDistanceFromInterpolationPoint(boolean isP1, Vec3 p) {
        if (isP1) {
            return this.p1In3d.p.distance(p);
        } else {
            return this.p2In3d.p.distance(p);
        }
    }

    public LinearInterpolator getPavement(boolean isLeft) {
        if (isLeft) {
            return this.leftPavement;
        } else {
            return this.rightPavement;
        }
    }

    @Override
    public Vec3 spawnCar(boolean rightLane, Vec3 p) {
        Vec3 endPoint;

        if (rightLane) {
            endPoint = this.p1In3d.n2;
        } else {
            endPoint = this.p1In3d.n1;
        }

        Vec3 movement = endPoint.subtract(this.p1In3d.p);
        Vec3 movementToMiddle = movement.multiply(0.5);

        Vec3 pointOnMiddleLane = computePoint(computeT(p));
        return pointOnMiddleLane.add(movementToMiddle);
    }

    @Override
    public double computeDistanceToFrontLeft(Vec3 p) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double computeDistanceToFrontRight(Vec3 p) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ArrayList<ControllerNode> convertToControllerList() {
        // TODO Auto-generated method stub
        return null;
    }
}
