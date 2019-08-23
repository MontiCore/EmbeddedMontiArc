/* (c) https://github.com/MontiCore/monticore */
package sensors;

import commons.controller.commons.BusEntry;
import java.util.List;

public class StaticPlannedTrajectoryXSensor extends StaticPlannedTrajectorySensor {

    public StaticPlannedTrajectoryXSensor(List<Double> trajectoryX) {
        super(BusEntry.PLANNED_TRAJECTORY_X, trajectoryX);
    }
}
