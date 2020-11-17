package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.Comparator;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;


/**
 * This class represents a simple LTL formula that can be evaluated/updated
 * at any time given an instance of the Vehicle class.
 *
 * The LTL formula of this class concentrate on vehicle's property such as
 * orientation, velocity, speed, battery, etc.
 *
 */
@Typed
public class MetricGoal extends Goal {
    private String property;
    private Comparator comparator;
    private double value;
    private String unit;

    public MetricGoal() {
    }

    public MetricGoal(LTLOperator ltlOperator, String property, Comparator comparator, double value, String unit) {
        this.ltlOperator = ltlOperator;
        this.property = property;
        this.comparator = comparator;
        this.value = value;
        this.unit = unit;
    }

    /**
     * Update the goal's status by evaluating the LTL formula.
     *
     * @param v an instance of Vehicle class
     */
    public void update(Vehicle v) {
        boolean result;
        if (property.equals(TrueVelocity.VALUE_NAME)) {
            double vel = (double) v.getPhysicalValues().getPhysicalValue(property).get();
            result = evaluate(vel);
        } else {
            System.out.printf("Vehicle is missing property %s.\n", this.property);
            return;
        }

        if (result) {
            updateStatus(TaskStatus.SUCCEEDED);
        } else {
            updateStatus(TaskStatus.FAILED);
        }
    }

    /**
     * Evaluate the boolean expression of this goal
     *
     * @param value
     * @return result of vehicle.[property] comparator value[unit]
     */
    private boolean evaluate(double value) {
        // TODO take unit into consideration
        if (comparator.equals(Comparator.EQUAL) && value == this.value) {
            return true;
        } else if (comparator.equals(Comparator.GREATER) && value > this.value) {
            return true;
        } else if (comparator.equals(Comparator.GREATER_EQUAL) && value >= this.value) {
            return true;
        } else if (comparator.equals(Comparator.LESS) && value < this.value) {
            return true;
        } else if (comparator.equals(Comparator.LESS_EQUAL) && value <= this.value) {
            return true;
        }
        return false;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this)
            return true;
        if (obj == null || obj.getClass() != getClass())
            return false;

        MetricGoal vg = (MetricGoal) obj;
        if (vg.getStatus() != status)
            return false;
        if (!vg.property.equals(property))
            return false;
        if (vg.value != value)
            return false;
        if (!vg.comparator.equals(comparator))
            return false;

        return true;
    }

    public static MetricGoalBuilder newBuilder() {
        return new MetricGoalBuilder();
    }

    public String getProperty() {
        return property;
    }

    public Comparator getComparator() {
        return comparator;
    }

    public double getValue() {
        return value;
    }

    public String getUnit() {
        return unit;
    }

}
