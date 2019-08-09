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

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import sensors.abstractsensors.AbstractSensor;
import org.apache.commons.math3.linear.RealVector;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 18-Dec-16.
 */

public class SpeedSensor extends AbstractSensor {

    private Double value;

    public SpeedSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
    }

    @Override
    public Double getValue() {
        return this.value;
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

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}