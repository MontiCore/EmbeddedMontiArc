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
import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;


public class FlexRayTest {
	
	
	@Test
	public void testSlotSize() {
		List<Object> components = new ArrayList<Object>();
		FlexRay flexRay = new FlexRay(components);
		flexRay.setMode(new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY));
		int expected = (int)Math.ceil((262 * 8)/(double)10);
		assertEquals(expected, flexRay.getSlotSize());
		
		flexRay.setMode(new FlexRayOperationMode(FlexRayOperationModeEnum.MAX_DATA));
		expected = (int)Math.ceil((262 * 8)/(double)20);
		assertEquals(expected, flexRay.getSlotSize());
	}
	
	@Test
	public void testCycleTime() {
		List<Object> components = new ArrayList<Object>();
		ModelicaPhysicalVehicle car = new ModelicaPhysicalVehicle();
		CameraSensor cam = new CameraSensor(car);
		CompassSensor com = new CompassSensor(car);
		SpeedSensor speed = new SpeedSensor(car);
		components.add(cam);
		components.add(com);
		components.add(speed);
		FlexRay flexRay = new FlexRay(components);
		int expected = flexRay.getSlotSize() * (3 + 4);
		assertEquals(expected, flexRay.getCycleTime());
		
	}
	
	
}
