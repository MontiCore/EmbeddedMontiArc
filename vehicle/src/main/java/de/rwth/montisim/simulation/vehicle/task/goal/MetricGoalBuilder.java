package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.utils.Comparator;

public class MetricGoalBuilder extends GoalBuilder<MetricGoalBuilder> {
    VehicleProperty property;
    Comparator comparator;
    double targetValue;
    String targetUnit;

    public MetricGoalBuilder setProperty(VehicleProperty property) {
        this.property = property;
        return getThis();
    }

    public MetricGoalBuilder lessEqual(double value, String unit) {
        comparator = Comparator.LESS_EQUAL;
        targetValue = value;
        targetUnit = unit;
        return getThis();
    }

    public MetricGoalBuilder less(double value, String unit) {
        comparator = Comparator.LESS;
        targetValue = value;
        targetUnit = unit;
        return getThis();
    }

    public MetricGoalBuilder greaterEqual(double value, String unit) {
        comparator = Comparator.GREATER_EQUAL;
        targetValue = value;
        targetUnit = unit;
        return getThis();
    }

    public MetricGoalBuilder greater(double value, String unit) {
        comparator = Comparator.GREATER;
        targetValue = value;
        targetUnit = unit;
        return getThis();
    }

    public MetricGoalBuilder equals(double value, String unit) {
        comparator = Comparator.EQUAL;
        targetValue = value;
        targetUnit = unit;
        return getThis();
    }

    @Override
    protected MetricGoalBuilder getThis() {
        return this;
    }

    @Override
    public Goal build() {
        return new MetricGoal(ltlOperator, property, comparator, targetValue, targetUnit);
    }
}
