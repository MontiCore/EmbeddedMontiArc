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

import simulation.EESimulator.EESimulator;
import commons.simulation.IPhysicalVehicle;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public abstract class AbstractDistanceSensor extends AbstractSensor {
    private Double value;

    public AbstractDistanceSensor(PhysicalVehicle physicalVehicle, EESimulator simulator) {
        super(physicalVehicle, simulator);
    }

    //TODO delete this
    public AbstractDistanceSensor(PhysicalVehicle vehicle) {
		super(vehicle);
	}

	@Override
    protected void calculateValue() {
        this.value = calculateDistance(getPhysicalVehicle());
    }

    protected abstract Double calculateDistance(IPhysicalVehicle v);

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}