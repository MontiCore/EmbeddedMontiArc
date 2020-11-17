package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.Vec2;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

public class PathGoalBuilder extends GoalBuilder<PathGoalBuilder> {
    final static double DEFAULT_RANGE = 10d;

    List<Vec2> coords;
    List<Coordinates> coordsLonLat;
    List<Long> coordsOsmId;
    Optional<Double> range;

    @Override
    protected PathGoalBuilder getThis() {
        return this;
    }

    @Override
    public Goal build() {
        if (coords != null) {
            return new PathGoal(ltlOperator, coords, range.orElse(DEFAULT_RANGE));
        } else if (coordsLonLat != null) {
            return new PathGoal(ltlOperator, range.orElse(DEFAULT_RANGE), coordsLonLat);
        } else if (coordsOsmId != null) {
            return new PathGoal(coordsOsmId, ltlOperator, range.orElse(DEFAULT_RANGE));
        }
        return null;
    }

    public PathGoalBuilder arrive(Vec2 coord) {
        if (coords == null) {
            coords = new LinkedList<>();
            coordsOsmId = null;
            coordsLonLat = null;
        }
        coords.add(coord);
        return getThis();
    }

    public PathGoalBuilder arrive(double x, double y) {
        if (coords == null) {
            coords = new LinkedList<>();
            coordsOsmId = null;
            coordsLonLat = null;
        }
        coords.add(new Vec2(x, y));
        return getThis();
    }

    public PathGoalBuilder arrive(Coordinates lonlat) {
        if (coordsLonLat == null) {
            coordsLonLat = new LinkedList<>();
            coordsOsmId = null;
            coords = null;
        }
        coordsLonLat.add(lonlat);
        return getThis();
    }

    public PathGoalBuilder arrive(long osmId) {
        if (coordsOsmId == null) {
            coordsOsmId = new LinkedList<>();
            coords = null;
            coordsLonLat = null;
        }
        coordsOsmId.add(osmId);
        return getThis();
    }

    public PathGoalBuilder withInRange(double rangeMeter) {
        range = Optional.of(rangeMeter);
        return getThis();
    }

}
