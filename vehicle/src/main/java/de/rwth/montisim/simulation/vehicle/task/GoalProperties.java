/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task;

import java.util.Optional;

import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public abstract class GoalProperties {
    public LTLOperator ltl_operator;

    public GoalProperties never() {
        ltl_operator = LTLOperator.NEVER;
        return this;
    }

    public GoalProperties always() {
        ltl_operator = LTLOperator.ALWAYS;
        return this;
    }

    public GoalProperties eventually() {
        ltl_operator = LTLOperator.EVENTUALLY;
        return this;
    }

    public GoalProperties until() {
        // TODO implement
        // this is only a placeholder
        ltl_operator = LTLOperator.UNTIL;
        return this;
    }

    public abstract Goal build(Vehicle vehicle, Optional<OsmMap> map, Optional<World> world);
}
