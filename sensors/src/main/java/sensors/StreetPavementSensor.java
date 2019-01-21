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
import commons.simulation.Sensor;
import sensors.abstractsensors.AbstractSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.environment.geometry.osmadapter.GeomStreet;
import simulation.environment.visualisationadapter.implementation.Street2D;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
//import sensors.factory.SensorFactory;
import simulation.vehicle.PhysicalVehicle;
//import simulation.vehicle.Vehicle;
//import static commons.controller.commons.BusEntry.SENSOR_STREETPAVEMENT;

import java.util.Optional;



/**
 * Created by Theresa on 08.01.2019.
 */


public class StreetPavementSensor extends AbstractSensor {

    private String value;

    public StreetPavementSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_CURRENT_SURFACE;
    }

    @Override
    public String getTypeName() {
        return String.class.getTypeName();
    }

    @Override
    protected void calculateValue() {
        World world = WorldModel.getInstance();
        GeomStreet geom = world.getStreet(getPhysicalVehicle());
        EnvStreet env = (EnvStreet) geom.getObject();
        Street2D s2d = (Street2D) env;
        this.value = s2d.getStreetPavement().toString();
    }



    @Override
    public String getValue() {
        return this.value;
    }


}