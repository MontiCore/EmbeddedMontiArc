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
 * Created by Aklima Zaman on 20-Jan-17.
 */
public class SteeringAngleSensor extends AbstractSensor {

    private Double value;

    public SteeringAngleSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                               HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
    }

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    @Override
    protected void calculateValue() {
        //NormalDistribution normalDistribution = new NormalDistribution(tempValue, .001);
        //tempValue = DoubleMath.mean(normalDistribution.sample(10));
        this.value = this.getPhysicalVehicle().getSteeringAngle();
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_STEERING;
    }
    
    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_STEERING;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}