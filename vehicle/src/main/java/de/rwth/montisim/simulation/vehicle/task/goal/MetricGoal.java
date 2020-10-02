package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.Comparator;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;


@Typed
public class MetricGoal extends Goal {
    private VehicleProperty property;
    private Comparator comparator;
    private double value;
    private String unit;

    public MetricGoal() {
    }

    public MetricGoal(LTLOperator ltlOperator, VehicleProperty property, Comparator comparator, double value, String unit) {
        this.ltlOperator = ltlOperator;
        this.property = property;
        this.comparator = comparator;
        this.value = value;
        this.unit = unit;
    }

    public void update(Vehicle v) {
        double currValue = 0;
        if (property.equals(VehicleProperty.SPEED)) {
            currValue = v.physicalObject.velocity.magnitude();
        }

        boolean result = compare(currValue);
        if (result) {
            updateStatus(TaskStatus.SUCCEEDED);
        } else {
            updateStatus(TaskStatus.FAILED);
        }
    }

    private boolean compare(double currValue) {
        // TODO take unit into consideration
        if (comparator.equals(Comparator.EQUAL) && currValue == value) {
            return true;
        } else if (comparator.equals(Comparator.GREATER) && currValue > value) {
            return true;
        } else if (comparator.equals(Comparator.GREATER_EQUAL) && currValue >= value) {
            return true;
        } else if (comparator.equals(Comparator.LESS) && currValue < value) {
            return true;
        } else if (comparator.equals(Comparator.LESS_EQUAL) && currValue <= value) {
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

    public VehicleProperty getProperty() {
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
