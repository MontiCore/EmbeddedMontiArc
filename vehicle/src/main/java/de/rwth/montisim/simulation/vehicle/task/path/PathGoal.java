/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task.path;

import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.task.Goal;

import java.util.List;
import java.util.ListIterator;
import java.util.Optional;
import java.util.Vector;


/**
 * PathGoal is used for setting up a set of locations, which a vehicle must or must
 * not reach. The order in the path matters, they will be considered in the exact
 * order as they are given.
 * <p>
 * For example, path = [node1, node2, ...] means node2
 * can only be considered, after node1 has been reached by the vehicle.
 */
public class PathGoal extends Goal {
    public transient final PathGoalProperties properties;
    public transient Vector<Vec2> path; // In local coordinates (meters)

    private int currDestIdx = -1; // the index of current destination (next location the vehicle should reach)

    public PathGoal(PathGoalProperties properties, Vector<Vec2> path) {
        super(properties);
        this.properties = properties;
        this.path = path;
    }


    public void update(Vehicle v) {
        switch (properties.ltl_operator) {
            case EVENTUALLY:
                // handled through "driveTarget"
                break;
            case NEVER:
                handleNever(v);
                break;
            default:
                throw new IllegalArgumentException("Specified LTL operator not supported for the PathGoal.");
        }
    }

    // Only gets called when this PathGoal is active
    // Returns true if all completed
    public boolean updateDriveTarget(Vehicle v, Optional<Navigation> nav) {
        if (!nav.isPresent())
            throw new IllegalArgumentException("PathGoal requires a Navigation component in the Vehicle.");
        Navigation navv = nav.get();
        boolean newTarget = false;
        if (currDestIdx < 0) {
            newTarget = true;
        } else if (currDestIdx < path.size()) {
            // If current drive goal finished -> pop
            Vec2 dest = path.get(currDestIdx);
            double dist = v.physicalObject.pos.asVec2().distance(dest);

            if (dist <= properties.range) {
                navv.popTargetPos(); // I think this is unnecessary now (since we set the whole path in the navigation), but doesn't hurt
                newTarget = true;
            }
        } else return true;
        if (newTarget) {
            currDestIdx += 1;
            if (currDestIdx < path.size()) {
                updateStatus(TaskStatus.RUNNING);
                Vector<Vec2> tmp = new Vector<>();
                // only get the points on the path, which we haven't visited yet
                for(int index = currDestIdx; index < path.size(); index++) {
                    tmp.add(path.get(index));
                }
                navv.setTargetsPos(tmp);
            } else {
                // goal is successful if all destinations have been reached
                updateStatus(TaskStatus.SUCCEEDED);
                return true;
            }
        }
        return false;
    }

    private void handleNever(Vehicle v) {
        // "never" operator expect the expression to always be false (FAILED)
        // i.e. the vehicle should never arrive in the area of any given destination
        TaskStatus status = TaskStatus.FAILED;

        for (Vec2 vec : path) {
            if (v.physicalObject.pos.distance(new Vec3(vec.x, vec.y, 0)) < properties.range) {
                // succeeded if arrived in the area of vec
                status = TaskStatus.SUCCEEDED;
                break;
            }
        }

        updateStatus(status);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this)
            return true;
        if (obj == null || obj.getClass() != getClass())
            return false;

        PathGoal goal = (PathGoal) obj;
        if (goal.getStatus() != status)
            return false;
        if (!goal.path.equals(path))
            return false;

        return true;
    }


    public List<Vec2> getPath() {
        return path;
    }


    public int getCurrDestIdx() {
        return currDestIdx;
    }
}

