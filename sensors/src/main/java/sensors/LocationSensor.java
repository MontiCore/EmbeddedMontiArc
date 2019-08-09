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
 * Created by Aklima Zaman on 20-Jan-17.
 */
public class LocationSensor extends AbstractSensor {

    private RealVector value;

    public LocationSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);

    }

    @Override
    public RealVector getValue() {
        return this.value;
    }

    @Override
    protected void calculateValue() {
        RealVector pos = getPhysicalVehicle().getPosition().copy();
        double x_axis = pos.getEntry(0);
        double y_axis = pos.getEntry(1);
        double z_axis = pos.getEntry(2); // we are ignoring z axis for now

        //NormalDistribution normalDistribution_x = new NormalDistribution(x_axis, 0.1);
        //x_axis = normalDistribution_x.sample();

        //NormalDistribution normalDistribution_y = new NormalDistribution(y_axis, 0.1);
        //y_axis = normalDistribution_y.sample();

        pos.setEntry(0, x_axis);
        pos.setEntry(1, y_axis);
        pos.setEntry(2, z_axis);

        this.value = pos;

    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_GPS_COORDINATES;
    }

    @Override
    public String getTypeName() {
        return RealVector.class.getTypeName();
    }
}