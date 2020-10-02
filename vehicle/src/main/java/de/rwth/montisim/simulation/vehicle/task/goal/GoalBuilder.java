package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;


public abstract class GoalBuilder<T extends GoalBuilder<T>> {
    public Goal fromJson(String json) throws SerializationException {
        return Json.instantiateFromJson(json, Goal.class);
    }

    public LTLOperator ltlOperator;

    protected abstract T getThis();

    public abstract Goal build();

    public T never() {
        ltlOperator = LTLOperator.NEVER;
        return (T) this;
    }

    public T always() {
        ltlOperator = LTLOperator.ALWAYS;
        return (T) this;
    }

    public T eventually() {
        ltlOperator = LTLOperator.EVENTUALLY;
        return (T) this;
    }

    public T until() {
        // TODO implement
        // this is only a placeholder
        ltlOperator = LTLOperator.UNTIL;
        return (T) this;
    }
}


