/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task;

import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.task.metric.MetricGoal;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoal;

import java.util.List;
import java.util.Optional;
import java.util.Vector;
import java.util.stream.Collectors;

public class Task {
    transient public final TaskProperties properties;
    transient protected final Optional<Navigation> nav;
    transient private Vector<Goal> goals = new Vector<>();
    transient private Vector<PathGoal> driveGoals = new Vector<>();
    int activeDriveGoal = -1;

    public Task(TaskProperties properties, Optional<Navigation> nav) {
        this.properties = properties;
        this.nav = nav;
    }

    public void update(Vehicle v) {
        goals.forEach(g -> g.update(v));
        // Check drive goals
        boolean newTarget = false;
        if (activeDriveGoal < 0) {
            newTarget = true;
        } else if (activeDriveGoal < driveGoals.size()) {
            newTarget = driveGoals.elementAt(activeDriveGoal).updateDriveTarget(v, nav);
        }
        if (newTarget) {
            ++activeDriveGoal;
            if (activeDriveGoal < driveGoals.size())
                driveGoals.elementAt(activeDriveGoal).updateDriveTarget(v, nav);
        }
    }

    protected void addGoal(Goal g) {
        goals.add(g);
        if (g instanceof PathGoal && g.properties.ltl_operator == LTLOperator.EVENTUALLY) {
            driveGoals.add((PathGoal) g);
        }
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

    public List<PathGoal> getPathGoals() {
        return goals.stream()
                .filter(g -> g instanceof PathGoal)
                .map(g -> (PathGoal) g)
                .collect(Collectors.toList());
    }

    public List<MetricGoal> getMetricGoals() {
        return goals.stream()
                .filter(g -> g instanceof MetricGoal)
                .map(g -> (MetricGoal) g)
                .collect(Collectors.toList());
    }
}
