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
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

/**
 * Created by Johannes on 12.09.2017.
 */
public class DayNightSensor extends AbstractSensor {
    public enum Daytime{
        Day,Night
    }

    private Daytime value;

    public DayNightSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                          HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId);
    }

    @Override
    protected void calculateValue() {
        this.value = Daytime.Day;
    }

    @Override
    public Daytime getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 1;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_DAYNIGHT;
    }
    
    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_DAYNIGHT;
    }

    @Override
    public String getTypeName() {
        return Daytime.class.getTypeName();
    }
}