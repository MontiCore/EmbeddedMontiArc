/* (c) https://github.com/MontiCore/monticore */
package sensors;

import commons.controller.commons.BusEntry;
import java.util.List;

public class StaticPlannedTrajectoryYSensor extends StaticPlannedTrajectorySensor {

    public StaticPlannedTrajectoryYSensor(List<Double> trajectoryY) {
        super(BusEntry.PLANNED_TRAJECTORY_Y, trajectoryY);
    }
}
