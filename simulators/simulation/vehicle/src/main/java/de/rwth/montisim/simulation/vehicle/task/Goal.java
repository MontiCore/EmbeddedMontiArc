/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task;

import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.vehicle.Vehicle;


/**
 * Goal is an abstraction of a LTL expression: LTLOperator expr.
 * <p>
 * Examples:
 * LTLOperator.NEVER expr: expr must never be true
 * LTLOperator.ALWAYS expr: expr must always be true
 * LTLOperator.EVENTUALLY expr: expr must be true before the simulation ends
 * <p>
 * Goal has a status which can be updated by calling update method while the simulation
 * is running.
 */
public abstract class Goal {
    transient public final GoalProperties properties;
    protected TaskStatus status = TaskStatus.RUNNING;

    public Goal(GoalProperties properties) {
        this.properties = properties;
    }

    public abstract void update(Vehicle v);


    /**
     * Update goal status regarding the LTL operator and the current value of the boolean
     * expression it contains.
     *
     * @param status new status
     */
    protected void updateStatus(TaskStatus status) {
        switch (properties.ltl_operator) {
            case ALWAYS:
                handleAlways(status);
                break;
            case EVENTUALLY:
                handleEventually(status);
                break;
            case NEVER:
                handleNever(status);
                break;
            default:
                throw new IllegalArgumentException("Specified LTL operator not supported.");
        }
    }

    private void handleAlways(TaskStatus status) {
        if (status == TaskStatus.FAILED)
            this.status = TaskStatus.FAILED;
        else if (status == TaskStatus.SUCCEEDED && this.status == TaskStatus.RUNNING)
            this.status = TaskStatus.SUCCEEDED;
    }

    private void handleNever(TaskStatus status) {
        if (status == TaskStatus.SUCCEEDED)
            this.status = TaskStatus.FAILED;
        else if (status == TaskStatus.FAILED && this.status != TaskStatus.FAILED)
            this.status = TaskStatus.SUCCEEDED;
    }

    private void handleEventually(TaskStatus status) {
        if (status == TaskStatus.SUCCEEDED)
            this.status = TaskStatus.SUCCEEDED;
    }

    public TaskStatus getStatus() {
        return status;
    }
}
