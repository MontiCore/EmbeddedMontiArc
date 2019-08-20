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
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Created by Aklima Zaman on 18-Dec-16.
 */

public class SpeedSensor extends AbstractSensor {

    private Double value;

    public SpeedSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                       HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator,subscribedMessages,targetsByMessageId);
    }

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    /**
     * Calculated velocity cannot be negative
     */
    @Override
    protected void calculateValue() {
        RealVector velocity = getPhysicalVehicle().getVelocity().copy();
        double velocityValue = velocity.getNorm();
        //NormalDistribution normalDistribution = new NormalDistribution(velocityValue, 0.1);
        //velocityValue = normalDistribution.sample();
        while (velocityValue < 0) {
            //velocityValue = normalDistribution.sample();
        }

        this.value = new Double(velocityValue);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_VELOCITY;
    }
    
    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_VELOCITY;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}