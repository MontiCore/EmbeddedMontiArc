/* (c) https://github.com/MontiCore/monticore */
package sensors;

import commons.controller.commons.BusEntry;
import commons.simulation.IPhysicalVehicle;
import simulation.EESimulator.EESimulator;
import sensors.abstractsensors.StaticPlannedTrajectorySensor;
import simulation.EESimulator.EEComponent;
import java.util.HashMap;
import java.util.List;

public class StaticPlannedTrajectoryYSensor extends StaticPlannedTrajectorySensor {


    public StaticPlannedTrajectoryYSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
            HashMap<BusEntry, List<EEComponent>> targetsByMessageId, List<Double> trajectoryY) {
    	super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId, trajectoryY);
    }

	public static BusEntry getSensorType() {
		return BusEntry.PLANNED_TRAJECTORY_Y;
	}

	@Override
	public BusEntry getType() {
		return BusEntry.PLANNED_TRAJECTORY_Y;
	}

	@Override
	protected void calculateValue() {
		// TODO Auto-generated method stub

	}
}
