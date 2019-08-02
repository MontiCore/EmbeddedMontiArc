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

import static org.junit.Assert.assertTrue;
import org.junit.Test;
import commons.simulation.Sensor;
import simulation.EESimulator.EESimulator;
import simulation.vehicle.PhysicalVehicle;
import simulation.vehicle.MassPointPhysicalVehicleBuilder;

import java.time.Instant;

/**
 * Created by Aklima Zaman on 19-Dec-16.
 */
public class SpeedSensorTest {

    @Test
    public void SpeedSensorTest() {
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        Sensor speedSensor = new SpeedSensor(physicalVehicle, simulator);

        assertTrue(!physicalVehicle.getVelocity().equals(speedSensor.getValue()));

    }
}