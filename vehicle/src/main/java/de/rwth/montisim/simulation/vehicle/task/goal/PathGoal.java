package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.util.List;


@Typed
public class PathGoal extends Goal {
    private List<Vec2> path;
    private double radius;
    private int currDestIdx;

    public PathGoal() {
    }

    public PathGoal(LTLOperator ltlOperator, List<Vec2> path, double radius) {
        this.ltlOperator = ltlOperator;
        this.path = path;
        this.radius = radius;
        this.currDestIdx = 0;
    }

    public void update(Vehicle v) {
        if (ltlOperator.equals(LTLOperator.EVENTUALLY)){
            handleEventually(v);
        } else if (ltlOperator.equals(LTLOperator.NEVER)) {
            handleNever(v);
        }
    }

    private void handleEventually(Vehicle v) {
        if (currDestIdx >= path.size()) return; // skip if all destinations have been reached

        Vec2 dest = path.get(currDestIdx);
        double dist = v.physicalObject.pos.asVec2().distance(dest);

        // nothing to update if current destination have not been reached yet
        if (dist > radius) {
            return;
        }

//        System.out.printf("%.2fm to target\n", dist);

        // a coordinate in path is reached,
        // update next destination and goal status
        currDestIdx += 1;
        if (currDestIdx < path.size()) {
            updateStatus(TaskStatus.RUNNING);
        } else {
            // goal is successful if all destinations have been reached
            updateStatus(TaskStatus.SUCCEEDED);
        }
    }

    private void handleNever(Vehicle v) {
        // "never" operator expect the expression to always be false (FAILED)
        // i.e. the vehicle should never arrive in the area of any given destination
        TaskStatus status = TaskStatus.FAILED;

        for (Vec2 vec: path) {
            if (v.physicalObject.pos.distance(new Vec3(vec.x, vec.y, 0)) < radius) {
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

    public static PathGoalBuilder newBuilder() {
        return new PathGoalBuilder();
    }

    public List<Vec2> getPath() {
        return path;
    }

    public double getRadius() {
        return radius;
    }

}

