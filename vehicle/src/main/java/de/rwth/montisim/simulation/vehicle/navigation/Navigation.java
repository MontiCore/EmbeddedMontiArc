/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.navigation;

import java.time.*;
import java.util.*;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.map.Path;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.*;
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
    // waypoint behind the vehicle and 9 after (targets)
    // In coordinate or local space (latlon - xy)
    // Might give less points if almost at target (or none if at target)
    public static final String TRAJECTORY_X_MSG = "trajectory_x";
    public static final String TRAJECTORY_Y_MSG = "trajectory_y";
    public static final String TRAJECTORY_LENGTH_MSG = "trajectory_length";
    public static final String TRAJECTORY_LON_MSG = "trajectory_lon";
    public static final String TRAJECTORY_LAT_MSG = "trajectory_lat";

    public static final BasicType AT_TARGET_POS_TYPE = BasicType.BOOLEAN;
    public static final BasicType CURRENT_TARGET_POS_TYPE = BasicType.VEC2;
    public static final int TRAJ_ARRAY_LENGTH = 10;
    public static final VectorType TRAJECTORY_X_TYPE = new VectorType(BasicType.DOUBLE, TRAJ_ARRAY_LENGTH);
    public static final VectorType TRAJECTORY_Y_TYPE = TRAJECTORY_X_TYPE;
    public static final VectorType TRAJECTORY_LON_TYPE = TRAJECTORY_X_TYPE;
    public static final VectorType TRAJECTORY_LAT_TYPE = TRAJECTORY_X_TYPE;

    //MessageInformation gpsPosMsg;
    transient MessageInformation truePosMsg;

    transient MessageInformation pushTargetPosMsg;
    transient MessageInformation popTargetPosMsg;

    transient MessageInformation atTargetPosMsg;
    transient MessageInformation currentTargetPosMsg;

    transient MessageInformation trajectoryXMsg;
    transient MessageInformation trajectoryYMsg;
    transient MessageInformation trajectoryLengthMsg;
    transient MessageInformation trajectoryLonMsg;
    transient MessageInformation trajectoryLatMsg;

    transient final Pathfinding pathfinding;

    final Stack<Vec2> targets = new Stack<>();
    Optional<Path> currentPath = Optional.empty();
    Optional<Vec2> currentPos = Optional.empty();
    transient Vec2 currentTraj[] = new Vec2[TRAJ_ARRAY_LENGTH];
    transient int currentTrajSize = 0;

    public Navigation(NavigationProperties properties, Pathfinding pathfinding) {
        super(properties);
        this.pathfinding = pathfinding;
        for (int i = 0; i < TRAJ_ARRAY_LENGTH; ++i) currentTraj[i] = new Vec2();
    }

    @Override
    protected void init() throws EEMessageTypeException {
        //this.gpsPosMsg = addInput(GPS_POS_MSG, BasicType.VEC2);
        this.truePosMsg = addInput(TruePosition.VALUE_NAME, TruePosition.TYPE);

        this.pushTargetPosMsg = addInput(PUSH_TARGET_POS_MSG, BasicType.VEC2, true, true);
        this.popTargetPosMsg = addInput(POP_TARGET_POS_MSG, BasicType.EMPTY, true, true);

        this.atTargetPosMsg = addOutput(AT_TARGET_POS_MSG, AT_TARGET_POS_TYPE);
        this.currentTargetPosMsg = addOutput(CURRENT_TARGET_POS_MSG, CURRENT_TARGET_POS_TYPE);

        this.trajectoryXMsg = addOutput(TRAJECTORY_X_MSG, TRAJECTORY_X_TYPE);
        this.trajectoryYMsg = addOutput(TRAJECTORY_Y_MSG, TRAJECTORY_Y_TYPE);
        this.trajectoryLengthMsg = addOutput(TRAJECTORY_LENGTH_MSG, BasicType.N);
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
        if (msg.isMsg(truePosMsg)) {
            currentPos = Optional.of((Vec2) msg.message);
            if (!currentPath.isPresent() && !targets.empty()){
                newTrajectory(time);
            } else {
                updateTrajectory(time);
            }
            // TODO
        } else if (msg.isMsg(pushTargetPosMsg)) {
            // Add new position to routing stack -> compute new routing
            // TODO
            pushTargetPos((Vec2)msg.message, time);
            newTrajectory(time);
        } else if (msg.isMsg(popTargetPosMsg)) {
            // Remove target from routing stack -> compute new routing
            // TODO
            popTargetPos();
            newTrajectory(time);
        }
    }

    public void pushTargetPos(Vec2 target){
        pushTargetPos(target, eesystem.simulator.getSimulationTime());
    }

    public void pushTargetPos(Vec2 target, Instant time){
        targets.push(target);
        sendMessage(time, currentTargetPosMsg, target);
    }
    
    public void popTargetPos() {
        currentTrajSize = 0;
        if (!targets.empty()) targets.pop();
        currentPath = Optional.empty();
    }

    private void newTrajectory(Instant time){
        currentPath = Optional.empty();
        if (!currentPos.isPresent()) return;
        if (targets.empty()){
            sendMessage(time, atTargetPosMsg, Boolean.TRUE);
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
        int size = Math.min(TRAJ_ARRAY_LENGTH, p.getLength()-index);
        currentTrajSize = size;
        double x[] = new double[TRAJ_ARRAY_LENGTH];
        double y[] = new double[TRAJ_ARRAY_LENGTH];
        for (int i = 0; i < size; ++i){
            x[i] = p.trajectoryX[index+i];
            y[i] = p.trajectoryY[index+i];
            currentTraj[i].x = x[i];
            currentTraj[i].y = y[i];
        }
        sendMessage(time, trajectoryLengthMsg, size);
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
                IPM.subtractTo(dir, point, lastPoint);
                // Manual normalization to keep the length
                double length = dir.magnitude();
                if (length > 0.001){
                    IPM.multiply(dir, 1/length);
                } else {
                    dir.set(Double.NaN,Double.NaN);
                }

                // 2) check if in segment bounds
                IPM.subtractTo(delta, pos, lastPoint);
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

    public Optional<Path> getCurrentPath() {
        return currentPath;
    }

    public Vec2[] getCurrentTraj() {
        return currentTraj;
    }

    public int getCurrentTrajSize() {
        return currentTrajSize;
    }

    public Stack<Vec2> getTargets() {
        return targets;
    }

}