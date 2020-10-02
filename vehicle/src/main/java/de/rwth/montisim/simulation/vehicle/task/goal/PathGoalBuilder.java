package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.utils.Vec2;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

public class PathGoalBuilder extends GoalBuilder<PathGoalBuilder> {
    final static double DEFAULT_RANGE = 10d;

    List<Vec2> coords;
    Optional<Double> range;

    @Override
    protected PathGoalBuilder getThis() {
        return this;
    }

    @Override
    public Goal build() {
        return new PathGoal(ltlOperator, coords, range.orElse(DEFAULT_RANGE));
    }

    public PathGoalBuilder arrive(Vec2 coord) {
        if (coords == null){
            coords = new LinkedList<>();
        }
        coords.add(coord);
        return getThis();
    }

    public PathGoalBuilder arrive(double x, double y) {
        if (coords == null){
            coords = new LinkedList<>();
        }
        coords.add(new Vec2(x, y));
        return getThis();
    }

    public PathGoalBuilder withInRange(double rangeMeter) {
        range = Optional.of(rangeMeter);
        return getThis();
    }

}
