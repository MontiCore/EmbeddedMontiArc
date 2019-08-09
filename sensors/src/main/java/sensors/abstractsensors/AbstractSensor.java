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

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.Sensor;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 1/20/2017.
 */
public abstract class AbstractSensor implements Sensor {

    private PhysicalVehicle physicalVehicle;

    public AbstractSensor(PhysicalVehicle physicalVehicle) {
        this.physicalVehicle = physicalVehicle;
    }


    @Override
    public void update() {
        calculateValue();
    }

    public PhysicalVehicle getPhysicalVehicle() {
        return this.physicalVehicle;
    }

    /**
     * This method do the sensor calculations
     */
    protected abstract void calculateValue();



}