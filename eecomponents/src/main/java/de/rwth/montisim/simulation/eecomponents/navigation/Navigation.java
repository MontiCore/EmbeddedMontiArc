/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.navigation;

import java.time.*;
import java.util.*;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.ArrayType.Dimensionality;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.environment.pathfinding.*;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.eesimulator.components.*;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.*;

// TODOS:
// Send Empty "Overwrite" trajectory on errors
// Only send trajectory every X seconds

public class Navigation extends EEComponent {
    //public static final String GPS_POS_MSG = "gps_pos";
    public static final String PUSH_TARGET_POS_MSG = "push_target_pos";
    public static final String POP_TARGET_POS_MSG = "pop_target_pos";

    public static final String AT_TARGET_POS_MSG = "at_target_pos";
    public static final String CURRENT_TARGET_POS_MSG = "current_target_pos";
    // The following messages are sent regularly by the Navigation and give the one
    // waypoint behind the vehicle and 4 after (targets)
    // In coordinate or local space (latlon - xy)
    // Might give less points if almost at target (or none if at target)
    public static final String TRAJECTORY_X_MSG = "trajectory_x";
    public static final String TRAJECTORY_Y_MSG = "trajectory_y";
    public static final String TRAJECTORY_LON_MSG = "trajectory_lon";
    public static final String TRAJECTORY_LAT_MSG = "trajectory_lat";

    public static final DataType AT_TARGET_POS_TYPE = DataType.BOOLEAN;
    public static final DataType CURRENT_TARGET_POS_TYPE = DataType.VEC2;
    public static final DataType TRAJECTORY_X_TYPE = new ArrayType(DataType.DOUBLE, Dimensionality.DYNAMIC, 10);
    public static final DataType TRAJECTORY_Y_TYPE = TRAJECTORY_X_TYPE;
    public static final DataType TRAJECTORY_LON_TYPE = TRAJECTORY_X_TYPE;
    public static final DataType TRAJECTORY_LAT_TYPE = TRAJECTORY_X_TYPE;

    //MessageInformation gpsPosMsg;
    MessageInformation truePosMsg;

    MessageInformation pushTargetPosMsg;
    MessageInformation popTargetPosMsg;

    MessageInformation atTargetPosMsg;
    MessageInformation currentTargetPosMsg;

    MessageInformation trajectoryXMsg;
    MessageInformation trajectoryYMsg;
    MessageInformation trajectoryLonMsg;
    MessageInformation trajectoryLatMsg;

    final Pathfinding pathfinding;

    final Stack<Vec2> targets = new Stack<>();
    Optional<Path> currentPath = Optional.empty();
    Optional<Vec2> currentPos = Optional.empty();

    public Navigation(NavigationProperties properties, Pathfinding pathfinding) {
        super(properties);
        this.pathfinding = pathfinding;
    }

    @Override
    protected void init() throws EEMessageTypeException {
        //this.gpsPosMsg = addInput(GPS_POS_MSG, DataType.VEC2);
        this.truePosMsg = addInput(TruePosition.VALUE_NAME, TruePosition.TYPE);

        this.pushTargetPosMsg = addInput(PUSH_TARGET_POS_MSG, DataType.VEC2, true, true);
        this.popTargetPosMsg = addInput(POP_TARGET_POS_MSG, DataType.EMPTY, true, true);

        this.atTargetPosMsg = addOutput(AT_TARGET_POS_MSG, AT_TARGET_POS_TYPE);
        this.currentTargetPosMsg = addOutput(CURRENT_TARGET_POS_MSG, CURRENT_TARGET_POS_TYPE);

        this.trajectoryXMsg = addOutput(TRAJECTORY_X_MSG, TRAJECTORY_X_TYPE);
        this.trajectoryYMsg = addOutput(TRAJECTORY_Y_MSG, TRAJECTORY_Y_TYPE);
        this.trajectoryLonMsg = addOutput(TRAJECTORY_LON_MSG, TRAJECTORY_LON_TYPE);
        this.trajectoryLatMsg = addOutput(TRAJECTORY_LAT_MSG, TRAJECTORY_LAT_TYPE);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        // TODO Auto-generated method stub
        Message msg = msgRecvEvent.getMessage();
        Instant time = msgRecvEvent.getEventTime();
        // if (msg.msgId == gpsPosMsg.messageId) {
        //     // Update position -> Check current routing
        //     // TODO
        // } else 
        if (msg.msgId == truePosMsg.messageId) {
            currentPos = Optional.of((Vec2) msg.message);
            if (!currentPath.isPresent() && !targets.empty()){
                newTrajectory(time);
            } else {
                updateTrajectory(time);
            }
            // TODO
        } else if (msg.msgId == pushTargetPosMsg.messageId) {
            // Add new position to routing stack -> compute new routing
            // TODO
            pushTargetPos((Vec2)msg.message, time);
            newTrajectory(time);
        } else if (msg.msgId == popTargetPosMsg.messageId) {
            // Remove target from routing stack -> compute new routing
            // TODO
            popTargetPos();
            newTrajectory(time);
        }
    }

    public void pushTargetPos(Vec2 target){
        pushTargetPos(target, simulator.getSimulationTime());
    }

    public void pushTargetPos(Vec2 target, Instant time){
        targets.push(target);
        sendMessage(time, currentTargetPosMsg, target);
    }
    
    public void popTargetPos() {
        if (!targets.empty()) targets.pop();
    }

    private void newTrajectory(Instant time){
        currentPath = Optional.empty();
        if (!currentPos.isPresent()) return;
        if (targets.empty()){
            sendMessage(time, atTargetPosMsg, new Boolean(true));
            return;
        }
        try {
            currentPath = Optional.of(pathfinding.findShortestPath(currentPos.get(), targets.peek()));
            updateTrajectory(time);
        } catch(Exception e){
            e.printStackTrace();
        }
    }

    public void updateTrajectory(Instant time) {
        if (!currentPos.isPresent()) return;
        if (!currentPath.isPresent()) return;
        int index = getNearestSegment(currentPos.get());
        if (index < 0) return;
        Path p = currentPath.get();
        int size = Math.min(10, p.getLength()-index);
        double x[] = new double[size];
        double y[] = new double[size];
        for (int i = 0; i < size; ++i){
            x[i] = p.trajectoryX[index+i];
            y[i] = p.trajectoryY[index+i];
        }
        sendMessage(time, trajectoryXMsg, x, 8*size);
        sendMessage(time.plus(Duration.ofMillis(10)), trajectoryYMsg, y, 8*size);
    }

    private int getNearestSegment(Vec2 pos) {
        double currentNearestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        Path p = currentPath.get();
        int count = p.getLength();
        Vec2 normal = new Vec2();
        Vec2 dir = new Vec2();
        Vec2 delta = new Vec2();
        Vec2 point = new Vec2();
        double dist;
        Vec2 lastPoint = new Vec2();
        boolean hasLastPoint = false;
        
        for (int i = 0; i < count; i++){
            p.get(i, point);

            if (hasLastPoint){
                // Check segment

                // 1) Get segment "normal"
                IPM.subtractToVec(point, lastPoint, dir);
                // Manual normalization to keep the length
                double length = dir.magnitude();
                if (length > 0.001){
                    IPM.multiply(dir, 1/length);
                } else {
                    dir.set(Double.NaN,Double.NaN);
                }

                // 2) check if in segment bounds
                IPM.subtractToVec(pos, lastPoint, delta);
                double projPos = IPM.dot(dir, delta);
                if (projPos > 0 && projPos < length) {
                
                    // 3) get distance
                    normal.set(-dir.y, dir.x);
                    dist = Math.abs(IPM.dot(normal, delta));
                    if (dist < currentNearestDistance){
                        currentNearestDistance = dist;
                        closestIndex = i-1;
                    }
                }
            }

            //Check point
            dist = point.distance(pos);
            if (dist < currentNearestDistance){
                currentNearestDistance = dist;
                closestIndex = i;
            }

            lastPoint.set(point);
            hasLastPoint = true;
        }
        return closestIndex;
    }


    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.SERVICE;
    }

    
}