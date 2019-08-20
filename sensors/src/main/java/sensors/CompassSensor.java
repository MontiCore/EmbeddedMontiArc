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
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

/**
 * Created by kirchhof on 10/03/2017.
 */
public class CompassSensor extends AbstractSensor {

    private Double value;

    public CompassSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                         HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_COMPASS;
    }
    
	public static BusEntry getSensorType() {
        return BusEntry.SENSOR_COMPASS;
	}

    @Override
    public Object getValue() {
        return value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }

    @Override
    protected void calculateValue() {
        RealMatrix rotation = getPhysicalVehicle().getRotation();
        RealVector yAxis = new ArrayRealVector(new double[] {0.0, 1.0, 0.0});
        RealVector v = rotation.operate(yAxis);
        //the angle in the xy-plane is needed
        v.setEntry(2, 0.0);

        double cos = v.cosine(yAxis);
        double angle = Math.acos(cos);

        //If the vector points to quadrant I or IV, ensure value range goes up to 2*pi
        if (v.getEntry(0) > 0) {
            angle = -angle;
            angle += 2*Math.PI;
        }
        value = angle;
    }




    }
