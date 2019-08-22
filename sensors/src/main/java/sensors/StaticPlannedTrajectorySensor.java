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
import commons.simulation.Sensor;
import org.apache.commons.lang3.Validate;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EESimulator;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class StaticPlannedTrajectorySensor extends AbstractSensor implements Sensor {

    private final BusEntry type;
    private final List<Double> trajectory;

    public StaticPlannedTrajectorySensor(BusEntry type, List<Double> trajectory, IPhysicalVehicle physicalVehicle, EESimulator simulator) {
        super(physicalVehicle, simulator, null, null);
        this.type = Validate.notNull(type);
        Validate.notNull(trajectory);
        List<Double> defensiveCopy = new ArrayList<>(trajectory);
        Validate.noNullElements(defensiveCopy);
        this.trajectory = Collections.unmodifiableList(defensiveCopy);
    }

    @Override
    public Object getValue() {
        return trajectory;
    }

    @Override
    public BusEntry getType() {
        return type;
    }

    @Override
    public String getTypeName() {
        return Collections.<Double>emptyList().getClass().getTypeName();
    }

    @Override
    public void update(Instant actualTime) {
    }

    @Override
    public int getDataLength(){
        return 6 * trajectory.size();
    }
}