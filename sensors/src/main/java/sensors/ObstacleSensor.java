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
import commons.simulation.PhysicalObject;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;
import simulation.EESimulator.EESimulator;
import simulation.vehicle.PhysicalVehicle;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by Marius on 12.09.2017.
 */
public class ObstacleSensor extends AbstractSensor {

    private List<PhysicalObject> result = Collections.synchronizedList(new LinkedList<>());
    private Object[] value = new Object[2];

    public ObstacleSensor(PhysicalVehicle physicalVehicle, EESimulator simulator,List<BusEntry> subscribedMessages,
                          HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
    }

    protected void calculateValue() {
        result.clear();
        //TODO: result.addAll Collections.synchronizedList (Alle physikalischen Objekte in einer Liste brauchen wir)
        value[0] = Double.MAX_VALUE;
        for(PhysicalObject k : result){
            Double t = getPhysicalVehicle().getGeometryPosition().getDistance(k.getGeometryPosition());
            if(((Double) value[0]) < t) {
                value[0]=t;
                value[1]=k.getVelocity();
            }
        }
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_OBSTACLE;
    }

    @Override
    public int getDataLength() {
        return 12;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }

    @Override
    public Object[] getValue() {
        return value;
    }
}