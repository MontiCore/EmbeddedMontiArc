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

import org.apache.commons.math3.distribution.NormalDistribution;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import sensors.abstractsensors.AbstractSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by zaman on 2/8/2017.
 */
public class WeatherSensor extends AbstractSensor {
    private Double value;

    public WeatherSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
    }

    @Override
    protected void calculateValue() {
        World world = WorldModel.getInstance();
        double weatherValue = world.getWeather();
        NormalDistribution normalDistribution = new NormalDistribution(weatherValue, 0.0003);
        this.value = new Double(normalDistribution.sample());
    }

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_WEATHER;
    }

    @Override public String getTypeName() {
        return Double.class.getTypeName();
    }
}