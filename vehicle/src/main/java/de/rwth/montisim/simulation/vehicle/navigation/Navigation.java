/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.navigation;

import java.time.*;
import java.util.*;
import java.util.stream.Collectors;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.map.Path;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
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
    transient int truePosMsg;

    transient int pushTargetPosMsg;
    transient int popTargetPosMsg;

    transient int atTargetPosMsg;
    transient int currentTargetPosMsg;

    transient int trajectoryXMsg;
    transient int trajectoryYMsg;
    transient int trajectoryLengthMsg;
    transient int trajectoryLonMsg;
    transient int trajectoryLatMsg;

    transient final Pathfinding pathfinding;

    final Stack<Vec2> targets = new Stack<>();
    Optional<Path> currentPath = Optional.empty();
    Optional<Vec2> currentPos = Optional.empty();
    transient Vec2 currentTraj[] = new Vec2[TRAJ_ARRAY_LENGTH];
    transient int currentTrajSize = 0;

    public Navigation(NavigationProperties properties, EESystem eeSystem, Pathfinding pathfinding) {
        super(properties, eeSystem);
        this.pathfinding = pathfinding;
        for (int i = 0; i < TRAJ_ARRAY_LENGTH; ++i) currentTraj[i] = new Vec2();


        //this.gpsPosMsg = addInput(GPS_POS_MSG, BasicType.VEC2);
        this.truePosMsg = addPort(PortInformation.newOptionalInputDataPort(TruePosition.VALUE_NAME, TruePosition.TYPE, false));

        this.pushTargetPosMsg = addPort(PortInformation.newOptionalInputDataPort(PUSH_TARGET_POS_MSG, BasicType.VEC2, true));
        this.popTargetPosMsg = addPort(PortInformation.newOptionalInputDataPort(POP_TARGET_POS_MSG, BasicType.EMPTY, true));

        this.atTargetPosMsg = addPort(PortInformation.newOptionalOutputDataPort(AT_TARGET_POS_MSG, AT_TARGET_POS_TYPE));
        this.currentTargetPosMsg = addPort(PortInformation.newOptionalOutputDataPort(CURRENT_TARGET_POS_MSG, CURRENT_TARGET_POS_TYPE));

        this.trajectoryLengthMsg = addPort(PortInformation.newOptionalOutputDataPort(TRAJECTORY_LENGTH_MSG, BasicType.N));
        this.trajectoryXMsg = addPort(PortInformation.newOptionalOutputDataPort(TRAJECTORY_X_MSG, TRAJECTORY_X_TYPE));
        this.trajectoryYMsg = addPort(PortInformation.newOptionalOutputDataPort(TRAJECTORY_Y_MSG, TRAJECTORY_Y_TYPE));
        this.trajectoryLonMsg = addPort(PortInformation.newOptionalOutputDataPort(TRAJECTORY_LON_MSG, TRAJECTORY_LON_TYPE));
        this.trajectoryLatMsg = addPort(PortInformation.newOptionalOutputDataPort(TRAJECTORY_LAT_MSG, TRAJECTORY_LAT_TYPE));
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
            if (!currentPath.isPresent() && !targets.empty()) {
                newTrajectory(time);
            } else {
                updateTrajectory(time);
            }
            // TODO
        } else if (msg.isMsg(pushTargetPosMsg)) {
            // Add new position to routing stack -> compute new routing
            // TODO
            pushTargetPos((Vec2) msg.message, time);
            newTrajectory(time);
        } else if (msg.isMsg(popTargetPosMsg)) {
            // Remove target from routing stack -> compute new routing
            // TODO
            popTargetPos();
            newTrajectory(time);
        }
    }

    /**
     * Set the whole path as targets. Will later calculate a trajectory through this given path.
     * @param targets the order of points to generate the trajectory through
     */
    public void setTargetsPos(Vector<Vec2> targets) { setTargetsPos(targets, eesystem.simulator.getSimulationTime()); }

    /**
     * Set the whole path as targets. Will later calculate a trajectory through this given path.
     * @param targets the order of points to generate the trajectory through
     * @param time
     */
    public void setTargetsPos(Vector<Vec2> targets, Instant time) {
        this.targets.clear();
        this.targets.addAll(targets);
        currentPath = Optional.empty();
        currentTrajSize = 0;
        sendMessage(time, currentTargetPosMsg, targets.firstElement());
    }

    public void pushTargetPos(Vec2 target) {
        pushTargetPos(target, eesystem.simulator.getSimulationTime());
    }

    public void pushTargetPos(Vec2 target, Instant time) {
        targets.push(target);
        currentPath = Optional.empty();
        currentTrajSize = 0;
        sendMessage(time, currentTargetPosMsg, target);
    }

    public void popTargetPos() {
        currentTrajSize = 0;
        if (!targets.empty()) targets.pop();
        currentPath = Optional.empty();
    }

    private void newTrajectory(Instant time) {
        currentPath = Optional.empty();
        if (!currentPos.isPresent()) return;
        if (targets.empty()) {
            sendMessage(time, atTargetPosMsg, Boolean.TRUE);
            return;
        }
        try {
            // currentPath = Optional.of(pathfinding.findShortestPath(currentPos.get(), targets.peek()));

            Path path = new Path(0);

            // create whole path by combining all sub paths between the adjacent targets
            Vec2 lastPos = currentPos.get();
            for(Vec2 target : targets) {
                Path subPath = pathfinding.findShortestPath(lastPos, target);

                Path newPath = new Path(path.getLength() + subPath.getLength());

                // copy existing path
                for (int index = 0; index < path.getLength(); index++) {
                    Vec2 tmp = new Vec2();
                    path.get(index, tmp);
                    newPath.set(index, tmp.x, tmp.y);
                }

                // append the sub path
                for (int index = 0; index < subPath.getLength(); index++) {
                    Vec2 tmp = new Vec2();
                    subPath.get(index, tmp);
                    newPath.set(index + path.getLength(), tmp.x, tmp.y);
                }

                path = newPath;

                lastPos = target;
            }

            currentPath = Optional.of(path);

            updateTrajectory(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void updateTrajectory(Instant time) {
        if (!currentPos.isPresent()) return;
        if (!currentPath.isPresent()) return;
        int index = getNearestSegment(currentPos.get());
        if (index < 0) return;
        Path p = currentPath.get();
        int size = Math.min(TRAJ_ARRAY_LENGTH, p.getLength() - index);
        double x[] = new double[TRAJ_ARRAY_LENGTH];
        double y[] = new double[TRAJ_ARRAY_LENGTH];

        int pointCount = 0;
        for (int i = 0; i < size; ++i) {
            double vx = p.trajectoryX[index + i];
            double vy = p.trajectoryY[index + i];
            // Only add the point if it is far enough from the last
            if (pointCount > 0) {
                double dist = currentTraj[pointCount - 1].distance(vx, vy);
                if (dist < 0.01) continue; // Skip
            }
            x[pointCount] = vx;
            y[pointCount] = vy;
            currentTraj[pointCount].x = x[pointCount];
            currentTraj[pointCount].y = y[pointCount];
            ++pointCount;
        }

        currentTrajSize = pointCount;
        sendMessage(time, trajectoryLengthMsg, currentTrajSize);
        sendMessage(time, trajectoryXMsg, x, 8 * currentTrajSize);
        sendMessage(time.plus(Duration.ofMillis(10)), trajectoryYMsg, y, 8 * currentTrajSize);
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

        for (int i = 0; i < count; i++) {
            p.get(i, point);

            if (hasLastPoint) {
                // Check segment

                // 1) Get segment "normal"
                IPM.subtractTo(dir, point, lastPoint);
                // Manual normalization to keep the length
                double length = dir.magnitude();
                if (length > 0.001) {
                    IPM.multiply(dir, 1 / length);
                } else {
                    dir.set(Double.NaN, Double.NaN);
                }

                // 2) check if in segment bounds
                IPM.subtractTo(delta, pos, lastPoint);
                double projPos = IPM.dot(dir, delta);
                if (projPos > 0 && projPos < length) {

                    // 3) get distance
                    normal.set(-dir.y, dir.x);
                    dist = Math.abs(IPM.dot(normal, delta));
                    if (dist < currentNearestDistance) {
                        currentNearestDistance = dist;
                        closestIndex = i - 1;
                    }
                }
            }

            //Check point
            dist = point.distance(pos);
            if (dist < currentNearestDistance) {
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

    public Optional<Integer> getCurrentPathIndex() {
        if (!currentPos.isPresent() || !currentPath.isPresent()) return Optional.empty();
        return Optional.of(getNearestSegment(currentPos.get()));
    }

    public Optional<Double> getRemainingPathLength() {
      Optional<Integer> index_optional = getCurrentPathIndex();
      if (!index_optional.isPresent()) {
        return Optional.empty();
      }
      Path p = currentPath.get();
      int index = index_optional.get();
      double distance = currentPos.get().distance(p.trajectoryX[index], p.trajectoryY[index]);
      for (int i = index; i < p.getLength() - 1; i++) {
        distance += (new Vec2(p.trajectoryX[i], p.trajectoryY[i])).distance(p.trajectoryX[i + 1], p.trajectoryY[i + 1]);
      }
      return Optional.of(distance);
    }
}