/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.splines;

import de.rwth.montisim.commons.utils.Vec3;
import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import de.rwth.montisim.simulation.environment.geometry.osmadapter.GeomStreet;
import de.rwth.montisim.simulation.environment.pedestrians.PedestrianStreetParameters;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvNode;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * This class is a helper class for pedestrians to calculate the movement.
 *
 * Note: nearly all calculations will be based on all splines of the street. The movement will always
 *       be between 2 spline points
 */
public class StreetInterpolator {

    private static final double TOLERANCE = 1E-9;

    private static Line getLineBetweenPoints(Vec3 p1, Vec3 p2) {
        Vector2D v1 = new Vector2D(p1.x, p1.y);
        Vector2D v2 = new Vector2D(p2.x, p2.y);
        return new Line(v1, v2, TOLERANCE);
    }

    /**
     * @param l1 first line
     * @param l2 second line
     * @return the intersection point of both lines (z will always be zero)
     */
    private Vec3 getIntersectionPoint(Line l1, Line l2) {
        Vector2D intersection = l1.intersection(l2);
        return new Vec3(
                intersection.getX(),
                intersection.getY(),
                0
        );
    }

    /**
     * This class holds all information about the movement between two EnvNode Points
     */
    private class MovementBetweenSplinePoints {

        /** Starting point of the pavement "left" */
        private Vec3 mBasePointPreLeft;
        /** Ending point of the pavement "left" */
        private Vec3 mBasePointNextLeft;

        /** Starting point of the pavement "right" */
        private Vec3 mBasePointPreRight;
        /** Ending point of the pavement "right" */
        private Vec3 mBasePointNextRight;

        /** Direction for walking on the pavement */
        private Vec3 mDirectionVector;
        /** Direction for crossing the street */
        private Vec3 mOrthogonalVector;

        /** Center point between the start and the end point */
        private Vec3 mCenterOfMovement;

        /** Line for the "left" pavement */
        private Line mLeftLine;
        /** Line for the "right" pavement */
        private Line mRightLine;

        /** Boolean indicating that at the start of the line the left is the shorter one */
        private boolean mPreShorterSideIsLeft;
        /** Boolean indicating that at the end of the line the left is the shorter one */
        private boolean mNextShorterSideIsLeft;

        /** The last point at which we can cross the street safely (side dependes on "mPreShorterSideIsLeft") */
        private Vec3 mPreLastCrossingPoint;
        /** The last point at which we can cross the street safely (side dependes on "mNextShorterSideIsLeft") */
        private Vec3 mNextLastCrossingPoint;

        MovementBetweenSplinePoints(Vec3 basePre, Vec3 baseNext) {
            mDirectionVector = baseNext.subtract(basePre).normalize();
            mDirectionVector.z = 0;
            mOrthogonalVector = new Vec3(
                    -mDirectionVector.y,
                    mDirectionVector.x,
                    0
            ).normalize();
            mCenterOfMovement = basePre.add(mDirectionVector.multiply(basePre.distance(baseNext) * 0.5));
        }
    }

    /** The street on which we want to move */
    private GeomStreet mBaseStreet;

    /**
     * List of all movement vectors between the nodes; if the street has n nodes, this list contains n-1 datasets
     * This means index 0 is for the movement between EnvNode 0 and EnvNode 1, index 1 for EnvNode 1 and EnvNode 2
     */
    private List<MovementBetweenSplinePoints> mSplineDirections;

    // These two are internal helpers for the movement and should probably be kept in the PedestrianStreetParameters
    private int mMovingDirection = -1;
    private Vec3 mCrossingStartPoint = null;

    //  The width of the street with pavements
    private double mStreetAndPavementWidth = 8;

    public StreetInterpolator(GeomStreet base) {
        mBaseStreet = base;
        mSplineDirections = new ArrayList<>();

        precomputePavements();
    }

    /**
     * This method will spawn a pedestrian on a random place for this street
     * @param random the random class to use
     * @return the generated parameters
     */
    public PedestrianStreetParameters spawnAtRandomLocation(Random random) {
        mMovingDirection = random.nextInt(mSplineDirections.size());
        MovementBetweenSplinePoints curMovement = mSplineDirections.get(mMovingDirection);

        double howFar = random.nextDouble();
        boolean direction = random.nextBoolean();
        boolean isOnLeftPavement = random.nextBoolean();

        Vec3 curPosition;

        // We need to differentiate between the left and the right side since they may not have the same length
        if (isOnLeftPavement) {
            double maxDistance = curMovement.mBasePointPreLeft.distance(curMovement.mBasePointNextLeft);
            curPosition = curMovement.mBasePointPreLeft.add(
                    curMovement.mDirectionVector.multiply(maxDistance * howFar)
            );
        } else {
            double maxDistance = curMovement.mBasePointPreRight.distance(curMovement.mBasePointNextRight);
            curPosition = curMovement.mBasePointPreRight.add(
                    curMovement.mDirectionVector.multiply(maxDistance * howFar)
            );
        }

        return new PedestrianStreetParameters(
                false,
                curPosition,
                direction,
                isOnLeftPavement
        );
    }

    /**
     * This method will precompute many movement vectors and borders to ensure we do not have to do the heavy lifting
     * during the simulation
     */
    private void precomputePavements() {
        List<EnvNode> envNodeList = mBaseStreet.getObject().getNodes();

        for (int i = 0; i < envNodeList.size() - 1; i++) {
            EnvNode n1 = envNodeList.get(i);
            EnvNode n2 = envNodeList.get(i + 1);

            MovementBetweenSplinePoints msp = new MovementBetweenSplinePoints(
                    n1.point,
                    n2.point
            );

            mSplineDirections.add(msp);

            msp.mBasePointPreLeft = n1.point.add(msp.mOrthogonalVector.multiply(mStreetAndPavementWidth / 2));
            msp.mBasePointPreRight = n1.point.add(msp.mOrthogonalVector.multiply(-mStreetAndPavementWidth / 2));
            msp.mBasePointNextLeft = n2.point.add(msp.mOrthogonalVector.multiply(mStreetAndPavementWidth / 2));
            msp.mBasePointNextRight = n2.point.add(msp.mOrthogonalVector.multiply(-mStreetAndPavementWidth / 2));

            msp.mLeftLine = getLineBetweenPoints(msp.mBasePointPreLeft, msp.mBasePointNextLeft);
            msp.mRightLine = getLineBetweenPoints(msp.mBasePointPreRight, msp.mBasePointNextRight);

            // Test if we can link this segment with a previous one; only works when we have already calculated one
            if (i > 0) {
                MovementBetweenSplinePoints preMsp = mSplineDirections.get(i - 1);

                Vec3 leftLineIntersection = getIntersectionPoint(preMsp.mLeftLine, msp.mLeftLine);
                Vec3 rightLineIntersection = getIntersectionPoint(preMsp.mRightLine, msp.mRightLine);

                preMsp.mBasePointNextLeft = leftLineIntersection;
                msp.mBasePointPreLeft = leftLineIntersection;
                preMsp.mBasePointNextRight = rightLineIntersection;
                msp.mBasePointPreRight = rightLineIntersection;

                // Now check which side of the pavement is the shorter one
                boolean leftIsFarerAway = leftLineIntersection.distance(msp.mCenterOfMovement)
                        > rightLineIntersection.distance(msp.mCenterOfMovement);

                msp.mPreShorterSideIsLeft = leftIsFarerAway;
                msp.mNextShorterSideIsLeft = leftIsFarerAway;

                if (leftIsFarerAway) {
                    msp.mPreLastCrossingPoint = msp.mBasePointPreRight
                            .add(msp.mOrthogonalVector.multiply(mStreetAndPavementWidth));
                    preMsp.mNextLastCrossingPoint = preMsp.mBasePointNextRight
                            .add(preMsp.mOrthogonalVector.multiply(mStreetAndPavementWidth));
                } else {
                    msp.mPreLastCrossingPoint = msp.mBasePointPreLeft
                            .add(msp.mOrthogonalVector.multiply(-mStreetAndPavementWidth));
                    preMsp.mNextLastCrossingPoint = preMsp.mBasePointNextLeft
                            .add(preMsp.mOrthogonalVector.multiply(-mStreetAndPavementWidth));
                }
            }
        }
    }

    /**
     * Will calculate the new movement of the pedestrian
     *
     * Note: at the moment the result of this method also depends on some internal variables
     * @param preParams the parameters in the frame before
     * @param distance how far can we move
     * @return the newly calculated parameters
     */
    public PedestrianStreetParameters calculateNewMovement(PedestrianStreetParameters preParams, double distance) {
        Vec3 curPosition = preParams.getPosition();
        boolean directionForward = preParams.isDirection();
        boolean isCrossingTheStreet = preParams.isCrossing();
        boolean isOnLeftPavement = preParams.isLeftPavement();

        if (mMovingDirection == -1) {
            directionForward = true;
            curPosition = mBaseStreet.getObject().getNodes().get(0).point.clone();
            mMovingDirection = 0;
        }

        MovementBetweenSplinePoints curMovement = mSplineDirections.get(mMovingDirection);

        // Test if we are allowed to cross the street
        if (isCrossingTheStreet &&
                (mCrossingStartPoint != null || canCrossStreetAtPosition(curPosition, isOnLeftPavement, curMovement))) {
            Vec3 crossingDirection = curMovement.mOrthogonalVector;

            // Set up the movement vector in the correct direction
            if (isOnLeftPavement) {
                crossingDirection = crossingDirection.multiply(-1);
            }

            // Set the starting and ending points
            if (mCrossingStartPoint == null) {
                mCrossingStartPoint = curPosition;
            }

            // Move as far as we can and test how far we have gone
            Vec3 endPosition = curPosition.add(crossingDirection.multiply(distance));
            double newDistance = mCrossingStartPoint.distance(endPosition);

            // If we have passed our destination point we will stop crossing the street
            // and reset all saved local helper variables
            if (newDistance >= mStreetAndPavementWidth) {
                endPosition = mCrossingStartPoint.add(crossingDirection.multiply(mStreetAndPavementWidth));
                isOnLeftPavement = !isOnLeftPavement;
                mCrossingStartPoint = null;
                isCrossingTheStreet = false;
            }

            return new PedestrianStreetParameters(
                    isCrossingTheStreet,
                    endPosition,
                    directionForward,
                    isOnLeftPavement
            );
        }

        // This code below will calculate a movement on the pavement (e.g no crossing)
        Vec3 direction = curMovement.mDirectionVector;
        Vec3 targetingPoint = getTargetingPoint(directionForward, isOnLeftPavement, curMovement);

        // Account for direction
        if (!directionForward) {
            direction = direction.multiply(-1);
        }

        // Second: Move as far as possible to the specific node
        Vec3 newPosition = curPosition.add(direction.multiply(distance));

        double preDistance = targetingPoint.distance(curPosition);
        double newDistance = targetingPoint.distance(newPosition);

        // Check if we need to turn around now since we reached the end of the street
        if (newDistance >= preDistance) {
            newPosition = targetingPoint;

            if ((mMovingDirection == 0 && !directionForward)
                    || (mMovingDirection == mSplineDirections.size() - 1 && directionForward)) {
                directionForward = !directionForward;
            } else {
                mMovingDirection = mMovingDirection + (directionForward ? 1 : -1);
            }
        }

        return new PedestrianStreetParameters(
                false,
                newPosition,
                directionForward,
                isOnLeftPavement
        );
    }

    /**
     * Returns the point we want to move to
     * @param directionForward the direction we want to move
     * @param isOnLeftPavement if we are on the left pavement
     * @param curMovement the current movement we are executing
     * @return the point calculated
     */
    private Vec3 getTargetingPoint(boolean directionForward, boolean isOnLeftPavement,
                                   MovementBetweenSplinePoints curMovement) {
        if (directionForward) {
            return isOnLeftPavement ? curMovement.mBasePointNextLeft : curMovement.mBasePointNextRight;
        } else {
            return isOnLeftPavement ? curMovement.mBasePointPreLeft : curMovement.mBasePointPreRight;
        }
    }

    /**
     * Checks if we should not cross the street here
     * @param curPosition our position
     * @param endPoint the point we want to reach
     * @param lastCrossingPoint the last safe crossing point
     * @return if we cannot cross here safely
     */
    private boolean isInNonCrossingZone(Vec3 curPosition, Vec3 endPoint, Vec3 lastCrossingPoint) {
        double distanceNonCrossing = endPoint.distance(lastCrossingPoint);
        double distanceToEnd = endPoint.distance(curPosition);

        return distanceToEnd <= distanceNonCrossing;
    }

    /**
     * Checks if we can cross the street safely
     * @param curPosition our position
     * @param isOnLeftPavement the side on which pavement we are currently
     * @param msp our movement atm
     * @return if we can safely cross here
     */
    private boolean canCrossStreetAtPosition(Vec3 curPosition, boolean isOnLeftPavement, MovementBetweenSplinePoints msp) {
        if (isOnLeftPavement) {
            if (msp.mPreShorterSideIsLeft && msp.mPreLastCrossingPoint != null
                    && isInNonCrossingZone(curPosition, msp.mBasePointPreLeft, msp.mPreLastCrossingPoint)) {
                return false;
            }

            if (msp.mNextShorterSideIsLeft && msp.mNextLastCrossingPoint != null
                    && isInNonCrossingZone(curPosition, msp.mBasePointNextLeft, msp.mNextLastCrossingPoint)) {
                return false;
            }
        } else {
            if (!msp.mPreShorterSideIsLeft && msp.mPreLastCrossingPoint != null
                    && isInNonCrossingZone(curPosition, msp.mBasePointPreRight, msp.mPreLastCrossingPoint)) {
                return false;
            }

            if (!msp.mNextShorterSideIsLeft && msp.mNextLastCrossingPoint != null
                    && isInNonCrossingZone(curPosition, msp.mBasePointNextRight, msp.mNextLastCrossingPoint)) {
                return false;
            }
        }

        return true;
    }
}
