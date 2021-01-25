package de.rwth.montisim.simulation.vehicle.navigation;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.VectorType;
import de.rwth.montisim.commons.utils.UMath;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.vehicle.lidar.Lidar;

import java.time.Instant;
import java.util.Optional;
import java.util.Vector;


public class SpeedLimitService extends EEComponent {

    private World world;
    private SpeedLimitServiceProperties properties;


    private double[] trajectoryX = null;
    private double[] trajectoryY = null;
    private int trajectoryLength = 0;

    //inputs
    transient int trajectoryXMsg,
            trajectoryYMsg,
            trajectoryLengthMsg;

    //outputs
    transient int upperSpeedLimitMsg,
        lowerSpeedLimitMsg;

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
        } else if(msg.isMsg(trajectoryXMsg)) {
            trajectoryX = (double[]) msg.message;
        } else if(msg.isMsg(trajectoryLengthMsg)) {
            trajectoryLength = (int) msg.message;
        }
    }


    private void compute(Instant time) {
        //System.out.println("testest");
        double[] maxSpeedArr = fetchUpperSpeedLimits();
        //sendMessage(time, upperSpeedLimitMsg, new double[]{0,0,0,0,0,0,0,0,0,0}, 8*trajectoryLength);
        sendMessage(time, upperSpeedLimitMsg, maxSpeedArr, 8*trajectoryLength);
    }

    private double[] fetchUpperSpeedLimits() {
        Vector<Way> ways = world.ways;
        double[] maxSpeedArr = new double[trajectoryLength];
        for (int i=0; i < trajectoryLength; i++){
            maxSpeedArr[i] = -1;
        }
        if ( trajectoryX != null && trajectoryLength > 1){
            PointInformation previousPointInfo = getPointInformation(world.ways, new Vec3(trajectoryX[0], trajectoryY[0], 0));
            for(int i = 1; i < trajectoryLength; i++) {
                Vec3 nextTrajPoint = new Vec3(trajectoryX[i], trajectoryY[i], 0);

                if (previousPointInfo.ways.size() == 1){
                    //previous trajectory point is a point but not an intersection
                    maxSpeedArr[i-1] = previousPointInfo.ways.get(0).maxSpeed;
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
                        maxSpeedArr[i-1] = mutualWay.get().maxSpeed;
                        previousPointInfo = getPointInformation(tmpWays,nextTrajPoint);
                    } else {
                        previousPointInfo = getPointInformation(world.ways, nextTrajPoint);
                    }
                }
            }
        }
        return maxSpeedArr;
    }

    private Optional<Way> getMutualWay(PointInformation pointAInfo, Vec3 pointB) {
        for (Way way : pointAInfo.ways){
            for (int i=0; i < way.points.size() - 1; i++) {
                Vec3 currentPoint = way.points.get(i);
                Vec3 nextPoint = way.points.get(i+1);
                if (currentPoint.equals(pointAInfo.coordinates) && nextPoint.equals(pointB)
                    || currentPoint.equals(pointB) && nextPoint.equals(pointAInfo.coordinates)) {
                    return Optional.of(way);
                }
            }
        }
        return Optional.empty();
    }



    /** @function: getNodeIdOfPoint
     * @return: the nodeId if the given @code{point} is an intersection, otherwise -1
     * */
    private PointInformation getPointInformation(Vector<Way> ways, Vec3 point){
        PointInformation pointInformation = new PointInformation();
        for (Way way : ways) {
            for (int i=0;i<way.points.size(); i++) {
                if (way.points.get(i).equals(point)) {
                    pointInformation.coordinates = point;
                    pointInformation.ways.add(way);
                    if (way.nodeID.get(i) != -1) {
                        pointInformation.node = world.nodes.get(way.nodeID.get(i));
                    }
                }
            }
        }
        return pointInformation;
    }

//    private Vector<Node> getWayNodes(World world, Way way) {
//        Vector<Node> wayNodes = new Vector<Node>();
//        for (int nodeId : way.nodeID){
//            wayNodes.add(world.nodes.get(nodeId));
//        }
//        return wayNodes;
//    }

//    private Optional<Way> getNextWay(Node node, double x, double y){
//        if (node.ways.size() != 0) {
//            for (Way way : node.ways){
//                if (getNodeIdOfPoint(getWayNodes(world, way),x,y).isPresent()) {
//                    return Optional.of(way);
//                }
//            }
//        }
//
//        return Optional.empty();
//    }

    /**
     * @Class: PointInformation
     * It provides information about a specific point from the world.*/
    private static class PointInformation{
        public Vec3 coordinates;
        public Node node;
        public Vector<Way> ways = new Vector<>();
    }
}
