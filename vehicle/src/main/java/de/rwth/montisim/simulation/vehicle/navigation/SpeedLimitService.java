package de.rwth.montisim.simulation.vehicle.navigation;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
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


    private Double[] trajectoryX = null;
    private Double[] trajectoryY = null;
    private int trajectoryLength = 0;

    //inputs
    transient int trajectoryXMsg,
            trajectoryYMsg,
            trajectoryLengthMsg;

    //outputs
    transient int upperSpeedLimitMsg,
        lowerSpeedLimitMsg;

    public SpeedLimitService(SpeedLimitServiceProperties properties, EESystem eeSystem, World world) {
        super(properties, eeSystem);
        this.world = world;

        this.trajectoryLengthMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_LENGTH_MSG, BasicType.N, false));
        this.trajectoryXMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_X_MSG, Navigation.TRAJECTORY_X_TYPE, false));
        this.trajectoryYMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_Y_MSG, Navigation.TRAJECTORY_Y_TYPE, false));
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        Instant time = msgRecvEvent.getEventTime();
        if (msg.isMsg(trajectoryYMsg)) {
            trajectoryY = (Double[]) msg.message;
            compute(time);
        } else if(msg.isMsg(trajectoryXMsg)) {
            trajectoryX = (Double[]) msg.message;
        } else if(msg.isMsg(trajectoryLengthMsg)) {
            trajectoryLength = (int) msg.message;
        }
    }


    private void compute(Instant time) {
        Double[] maxSpeedArr = fetchUpperSpeedLimits();
        sendMessage(time, upperSpeedLimitMsg, maxSpeedArr);
    }

    private Double[] fetchUpperSpeedLimits() {
        Vector<Way> ways = world.ways;
        Double[] maxSpeedArr = new Double[trajectoryLength - 1];
        for (int i=0; i < trajectoryLength - 1; i++){
            maxSpeedArr[i] = Double.MAX_VALUE;
        }
        if ( trajectoryX != null && trajectoryLength > 1){
            Vector<Node> wayNodes = new Vector<Node>();
            for(Way way : ways){
                for (int nodeId : way.nodeID){
                    wayNodes.add(world.nodes.get(nodeId));
                }
            }
            for(int i = 0; i<trajectoryLength; i++){
                Optional<Node> node = getNodeForCoordinates(wayNodes, trajectoryX[i], trajectoryY[i]);
                if (node.isPresent()) {
                    Optional<Way> way = getNextWay(node.get(), trajectoryX[i+1], trajectoryY[i+1]);
                    if (way.isPresent()) {
                        maxSpeedArr[i] = way.get().maxSpeed;
                    } else {
                        break;
                    }
                }
            }
        }
        return maxSpeedArr;
    }

    private Optional<Node> getNodeForCoordinates(Vector<Node> nodes, double x, double y){
        for (Node node : nodes) {
            double currentDistance = node.point.distance(new Vec3(x,y,0));
            if (UMath.equalsThreshold(currentDistance, 0, Lidar.DOUBLE_TOLERANCE)){
                return Optional.of(node);
            }
        }
        return Optional.empty();
    }

    private Vector<Node> getWayNodes(World world, Way way) {
        Vector<Node> wayNodes = new Vector<Node>();
        for (int nodeId : way.nodeID){
            wayNodes.add(world.nodes.get(nodeId));
        }
        return wayNodes;
    }

    private Optional<Way> getNextWay(Node node, double x, double y){
        if (node.ways.size() != 0) {
            for (Way way : node.ways){
                if (getNodeForCoordinates(getWayNodes(world, way),x,y).isPresent()) {
                    return Optional.of(way);
                }
            }
        }

        return Optional.empty();
    }
}
