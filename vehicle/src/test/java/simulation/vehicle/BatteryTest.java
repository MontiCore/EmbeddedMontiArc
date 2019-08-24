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
package simulation.vehicle;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.junit.*;
import simulation.environment.WorldModel;
import simulation.environment.visualisationadapter.interfaces.EnvStreet.StreetPavements;

public class BatteryTest {
	Vehicle v;
	Battery battery;

	@Before
	public void setAll() {
		v = new MassPointPhysicalVehicle().getSimulationVehicle();
		battery = new Battery(v, 1000);
		v.setBattery(battery);

        VehicleActuator steering = v.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        VehicleActuator throttle = v.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);
        throttle.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MAX);

        VehicleActuator gear = v.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR);
        gear.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_GEAR_MAX);
	}
	
	// @Test
	// public void fullBattery() {
	// 	Assert.assertEquals(v.getBattery().getBatteryPercentage(), 100.0, 0.1);
	// }

	// @Test
	// public void testBatteryConsumption() {
	// 	v.setBattery(new Battery(v,1000));
	// 	v.getBattery().discharge();
	// 	Assert.assertEquals(900, v.getBattery().getCurrentCapacity(), 1.0);

	// 	v.getBattery().discharge();
	// 	Assert.assertEquals(800, v.getBattery().getCurrentCapacity(), 1.0);
	// }
}