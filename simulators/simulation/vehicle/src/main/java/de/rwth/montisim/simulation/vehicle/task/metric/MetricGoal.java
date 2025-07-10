/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task.metric;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.task.Goal;


/**
 * This class represents a simple LTL formula that can be evaluated/updated
 * at any time given an instance of the Vehicle class.
 * <p>
 * The LTL formula of this class concentrate on vehicle's property such as
 * orientation, velocity, speed, battery, etc.
 */
public class MetricGoal extends Goal {
    public static final double EPSILON = 0.000001; // For equality comparison
    transient public final MetricGoalProperties properties;
    transient final PhysicalValueDouble value;

    public MetricGoal(MetricGoalProperties properties, Vehicle vehicle) {
        super(properties);
        this.properties = properties;
        PhysicalValue val = vehicle.getPhysicalValues().getPhysicalValue(properties.compare);
        if (!(val instanceof PhysicalValueDouble)) {
            throw new IllegalArgumentException("MetricGoal only support physical values of type 'double' but '" + properties.compare + "' is not.");
        }
        this.value = (PhysicalValueDouble) val;
    }

    /**
     * Update the goal's status by evaluating the LTL formula.
     *
     * @param v an instance of Vehicle class
     */
    public void update(Vehicle v) {
        updateStatus(evaluate(value.getValue()) ? TaskStatus.SUCCEEDED : TaskStatus.FAILED);
    }

    /**
     * Evaluate the boolean expression of this goal
     *
     * @param value
     * @return result of vehicle.[property] comparator value[unit]
     */
    private boolean evaluate(double value) {
        switch (properties.operator) {
            case EQUAL:
                return properties.with - EPSILON <= value && value <= properties.with + EPSILON;
            case GREATER:
                return value > properties.with;
            case GREATER_EQUAL:
                return value >= properties.with;
            case LESS:
                return value < properties.with;
            case LESS_EQUAL:
                return value <= properties.with;
            default:
                return false;
        }
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this)
            return true;
        if (obj == null || obj.getClass() != getClass())
            return false;

        MetricGoal vg = (MetricGoal) obj;

        return status.equals(vg.status) &&
                properties.equals(vg.properties);
    }

}
