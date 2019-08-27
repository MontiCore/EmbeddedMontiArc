/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import java.util.List;

public class StaticPlannedTrajectoryYSensor extends StaticPlannedTrajectorySensor {

    public StaticPlannedTrajectoryYSensor(List<Double> trajectoryY) {
        super(BusEntry.PLANNED_TRAJECTORY_Y, trajectoryY);
    }
}
