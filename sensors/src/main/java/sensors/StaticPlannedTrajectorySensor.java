/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.Sensor;
import org.apache.commons.lang3.Validate;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class StaticPlannedTrajectorySensor implements Sensor {

    private final BusEntry type;
    private final List<Double> trajectory;

    public StaticPlannedTrajectorySensor(BusEntry type, List<Double> trajectory) {
        this.type = Validate.notNull(type);
        Validate.notNull(trajectory);
        List<Double> defensiveCopy = new ArrayList<>(trajectory);
        Validate.noNullElements(defensiveCopy);
        this.trajectory = Collections.unmodifiableList(defensiveCopy);
    }

    @Override
    public Object getValue() {
        return trajectory;
    }

    @Override
    public BusEntry getType() {
        return type;
    }

    @Override
    public String getTypeName() {
        return Collections.<Double>emptyList().getClass().getTypeName();
    }

    @Override
    public void update() {
    }
}
