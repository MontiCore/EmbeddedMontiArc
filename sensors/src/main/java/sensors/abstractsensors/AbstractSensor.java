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
package sensors.abstractsensors;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.*;

import java.util.HashMap;
import java.util.List;

import commons.simulation.IPhysicalVehicle;
import commons.simulation.Sensor;

/**
 * Created by Aklima Zaman on 1/20/2017.
 */
public abstract class AbstractSensor extends ImmutableEEComponent implements Sensor {
	
	private final IPhysicalVehicle physicalVehicle;

	public AbstractSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
			HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
		super(simulator, EEComponentType.SENSOR, subscribedMessages, targetsByMessageId);
		this.physicalVehicle = physicalVehicle;
	}

	@Override
	public void update() {
		calculateValue();
		
	}

	/**
	 * Return the physicalVehicle this sensor belongs to
	 * @return
	 */
	public IPhysicalVehicle getPhysicalVehicle() {
		return physicalVehicle;
	}

	/**
	 * This method do the sensor calculations
	 */
	protected abstract void calculateValue();

	/**
	 *
	 * @return individual lenght of the data of the sensor in bytes
	 */
	public abstract int getDataLength();

	@Override
	public void processEvent(EEDiscreteEvent event) {
		// TODO implement
	}

}