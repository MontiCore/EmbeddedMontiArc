/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task.metric;

import java.util.Optional;

import de.rwth.montisim.commons.utils.Comparator;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.task.GoalProperties;

@Typed("metric")
public class MetricGoalProperties extends GoalProperties {
    public String compare; // Which PhysicalValue this evaluates
    public double with = 0.0;
    public String unit = "";
    public Comparator operator = Comparator.EQUAL;


    // public MetricGoal(LTLOperator ltlOperator, String property, Comparator comparator, double value, String unit) {
    //     this.ltlOperator = ltlOperator;
    //     this.property = property;
    //     this.comparator = comparator;
    //     this.value = value;
    //     this.unit = unit;
    // }

    public MetricGoalProperties compare(String property) {
        this.compare = property;
        return this;
    }

    public MetricGoalProperties with(double value, String unit) {
        this.with = value;
        this.unit = unit;
        return this;
    }

    public MetricGoalProperties operator(Comparator comp) {
        this.operator = comp;
        return this;
    }

    @Override
    public MetricGoal build(Vehicle vehicle, Optional<OsmMap> map, Optional<World> world) {
        return new MetricGoal(this, vehicle);
    }

    @Override
    public boolean equals(Object o) {
        if (o == this)
            return true;
        if (o == null || o.getClass() != getClass())
            return false;
        MetricGoalProperties op = (MetricGoalProperties) o;
        return compare.equals(op.compare) &&
                with == op.with &&
                unit.equals(op.unit) &&
                operator == op.operator &&
                ltl_operator == op.ltl_operator;
    }
}
