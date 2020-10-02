package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.LTLOperator;
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
    TaskStatus status = TaskStatus.RUNNING;
    protected LTLOperator ltlOperator;

    public abstract void update(Vehicle v);


    /**
     * Update goal status depending on the LTL operator and the new status
     *
     * @param status new status
     */
    void updateStatus(TaskStatus status) {
        if (ltlOperator == LTLOperator.ALWAYS) {
            handleAlways(status);
        } else if (ltlOperator == LTLOperator.NEVER) {
            handleNever(status);
        } else if (ltlOperator == LTLOperator.EVENTUALLY) {
            handleEventually(status);
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

    public LTLOperator getLtlOperator() {
        return ltlOperator;
    }

    public TaskStatus getStatus() {
        return status;
    }
}
