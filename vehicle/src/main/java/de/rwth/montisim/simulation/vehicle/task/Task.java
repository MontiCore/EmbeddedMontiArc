package de.rwth.montisim.simulation.vehicle.task;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.task.goal.*;

import java.util.LinkedList;
import java.util.List;

@Typed
public class Task {
    private List<Goal> goals;

    public Task() {
        goals = new LinkedList<>();
    }

    public void addGoal(Goal goal) {
        goals.add(goal);
    }

    public void update(Vehicle v) {
        goals.forEach(g -> g.update(v));
    }

    public TaskStatus status() {
        TaskStatus status = TaskStatus.SUCCEEDED;

        for (Goal g : goals) {
            TaskStatus goalStatus = g.getStatus();
            if (goalStatus == TaskStatus.RUNNING) {
                status = TaskStatus.RUNNING;
            } else if (goalStatus == TaskStatus.FAILED) {
                status = TaskStatus.FAILED;
                break;
            }
        }

        return status;
    }

    public List<Goal> getGoals() {
        return goals;
    }

}
