/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package sensors;

import commons.controller.commons.BusEntry;
import commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.StaticPlannedTrajectorySensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

public class StaticPlannedTrajectoryXSensor extends StaticPlannedTrajectorySensor {

    public StaticPlannedTrajectoryXSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
            HashMap<BusEntry, List<EEComponent>> targetsByMessageId, List<Double> trajectoryX) {
    	super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId, trajectoryX);
    }
    
	public static BusEntry getSensorType() {
		return BusEntry.PLANNED_TRAJECTORY_X;
	}

	@Override
	public BusEntry getType() {
		return BusEntry.PLANNED_TRAJECTORY_X;
	}

	@Override
	protected void calculateValue() {
		// TODO Auto-generated method stub
		
	}

}