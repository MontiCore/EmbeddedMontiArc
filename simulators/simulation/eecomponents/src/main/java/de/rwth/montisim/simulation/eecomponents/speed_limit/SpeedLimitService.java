package de.rwth.montisim.simulation.eecomponents.speed_limit;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.VectorType;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

import java.time.Instant;
import java.util.Arrays;
import java.util.Optional;
import java.util.Vector;

/**
 * This component supplies the vehicle with the upper speed limits for the current trajectory.
 */
public class SpeedLimitService extends EEComponent {

    /**
     * The simulated world. Used for fetching speed limits
     */
    private final World world;

    private double[] trajectoryX = null;
    private double[] trajectoryY = null;
    private int trajectoryLength = 0;
    private double[] currentSpeedLimits = null;

    //input ports
    transient int trajectoryXMsg,
            trajectoryYMsg,
            trajectoryLengthMsg;

    //output ports
    transient int upperSpeedLimitMsg;

    public static final String UPPER_SPEED_LIMIT_MSG = "upper_speed_limit";

    public static final VectorType SPEED_LIMIT_TYPE = new VectorType(BasicType.DOUBLE, Navigation.TRAJ_ARRAY_LENGTH - 1);

    public SpeedLimitService(SpeedLimitServiceProperties properties, EESystem eeSystem, World world) {
        super(properties, eeSystem);
        this.world = world;

        this.trajectoryLengthMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_LENGTH_MSG, BasicType.N, false));
        this.trajectoryXMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_X_MSG, Navigation.TRAJECTORY_X_TYPE, false));
        this.trajectoryYMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_Y_MSG, Navigation.TRAJECTORY_Y_TYPE, false));

        this.upperSpeedLimitMsg = addPort(PortInformation.newRequiredOutputDataPort(UPPER_SPEED_LIMIT_MSG, SPEED_LIMIT_TYPE));
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        Instant time = msgRecvEvent.getEventTime();
        if (msg.isMsg(trajectoryYMsg)) {
            trajectoryY = (double[]) msg.message;
            compute(time);
        } else if (msg.isMsg(trajectoryXMsg)) {
            trajectoryX = (double[]) msg.message;
        } else if (msg.isMsg(trajectoryLengthMsg)) {
            trajectoryLength = (int) msg.message;
        }
    }

    /**
     * Main compute method of this component. It is executed each time a trajectory is received.
     *
     * @param time the event time of the last trajectoryY event
     */
    private void compute(Instant time) {
        double[] maxSpeedArr = fetchUpperSpeedLimits();
        sendMessage(time, upperSpeedLimitMsg, maxSpeedArr, 8 * (trajectoryLength - 1));
    }

    /**
     * Tries to fetch the upper speed limits for all the trajectory segments. Each segment is mapped
     * to its speed limit and if it doesn't have one, -1 is returned.
     *
     * @return array of speed limits (with a length of trajectoryLength - 1)
     */
    public double[] fetchUpperSpeedLimits() {
        double[] maxSpeedArr = new double[Navigation.TRAJ_ARRAY_LENGTH - 1];
        for (int i = 0; i < trajectoryLength - 1; i++) {
            maxSpeedArr[i] = -1;
        }
        if (trajectoryX != null && trajectoryLength > 1) {

            PointInformation previousPointInfo = getPointInformation(world.ways, new Vec3(trajectoryX[0],
                    trajectoryY[0], 0));

            for (int i = 1; i < trajectoryLength; i++) {
                Vec3 nextTrajPoint = new Vec3(trajectoryX[i], trajectoryY[i], 0);

                if (previousPointInfo.ways.size() == 1) {
                    //previous trajectory point is a point but not an intersection
                    maxSpeedArr[i - 1] = previousPointInfo.ways.get(0).maxSpeed;
                    previousPointInfo = getPointInformation(previousPointInfo.ways, nextTrajPoint);
                } else if (previousPointInfo.ways.size() == 0) {
                    //previous trajectory point cannot be found on any way
                    previousPointInfo = getPointInformation(world.ways, nextTrajPoint);
                } else {
                    // initial/previous trajectory point is an intersection
                    Optional<Way> mutualWay = getMutualWay(previousPointInfo, nextTrajPoint);

                    if (mutualWay.isPresent()) {
                        // current - and next trajectory points are on the same way
                        Vector<Way> tmpWays = new Vector<>();
                        tmpWays.add(mutualWay.get());
                        maxSpeedArr[i - 1] = mutualWay.get().maxSpeed;
                        previousPointInfo = getPointInformation(tmpWays, nextTrajPoint);
                    } else {
                        previousPointInfo = getPointInformation(world.ways, nextTrajPoint);
                    }
                }
            }
        }
        currentSpeedLimits = maxSpeedArr;
        return maxSpeedArr;
    }

    /**
     * Checks if two points lie on the same way directly behind each other. If they do, the way is returned, otherwise
     * an Optional.Empty() is returned. Only the ways in the pointAInfo object are considered.
     * If two ways which fulfill the condition exist, only the first one is returned.
     *
     * @param pointAInfo point information for a point A
     * @param pointB     point B
     * @return optional of the way, where the two points are contained.
     */
    private Optional<Way> getMutualWay(PointInformation pointAInfo, Vec3 pointB) {
        for (Way way : pointAInfo.ways) {
            for (int i = 0; i < way.points.size() - 1; i++) {
                Vec3 currentPoint = way.points.get(i);
                Vec3 nextPoint = way.points.get(i + 1);
                if (currentPoint.equals(pointAInfo.coordinates) && nextPoint.equals(pointB)
                        || currentPoint.equals(pointB) && nextPoint.equals(pointAInfo.coordinates)) {
                    return Optional.of(way);
                }
            }
        }
        return Optional.empty();
    }


    /**
     * Collects information about a point from a given vector of ways. The information
     * is returned as a PointInformation object
     *
     * @param point the point to collect information about
     * @param ways  Some ways that can, but don't have to contain the point
     * @return a PointInformation object with the corresponding coordinates, ways and node
     */
    private PointInformation getPointInformation(Vector<Way> ways, Vec3 point) {
        PointInformation pointInformation = new PointInformation();
        for (Way way : ways) {
            for (int i = 0; i < way.points.size(); i++) {
                if (way.points.get(i).equals(point)) {
                    pointInformation.coordinates = point;
                    pointInformation.ways.add(way);
                    if (way.nodeID.get(i) != -1) {
                        pointInformation.node = world.nodes.get(way.nodeID.get(i));
                        pointInformation.ways.addAll(pointInformation.node.ways);
                    }
                }
            }
        }
        return pointInformation;
    }

    /**
     * @param index the position of the desired speed limit within the speed limit array
     * @return entry of speed limit array at given index
     */
    public double getSpeedLimit(int index) {
        if (currentSpeedLimits == null) return -1.0;
        if (index >= currentSpeedLimits.length) return -2.0;
        return currentSpeedLimits[index];
    }

    /**
     * @return array containing currently available speed limits
     */
    public double[] getSpeedLimits() {
        if (currentSpeedLimits == null) {
            double[] temp = new double[Navigation.TRAJ_ARRAY_LENGTH - 1];
            Arrays.fill(temp, -1.0);
            return temp;
        }
        return currentSpeedLimits;
    }

    /**
     * PointInformation
     * It provides information about a specific point from the world.
     */
    private static class PointInformation {
        /**
         * The coordinates of the point
         */
        public Vec3 coordinates;

        /**
         * A node corresponding to the point. Not every point is correlated
         * to a node, so this can be null.
         */
        public Node node;

        /**
         * A vector of ways that contain the point.
         */
        public Vector<Way> ways = new Vector<>();
    }

}
