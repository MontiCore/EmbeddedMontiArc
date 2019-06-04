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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static commons.controller.commons.BusEntry.*;
import commons.simulation.DiscreteEventSimulationNotifiable;
import org.junit.Test;

import sensors.CameraSensor;
import sensors.CompassSensor;
import sensors.SpeedSensor;
import simulation.vehicle.ModelicaPhysicalVehicle;


public class FlexRayTest {
	/**
	 * Test the public void setMode(FlexrayOperationMode) function
	 */
	@Test
	public void testMode() {
		//Flexray Initialisaiton
		FlexRay flexray = new FlexRay();

		//Mode setup
		FlexRayOperationMode modeOne = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);
		FlexRayOperationMode modeTwo = new FlexRayOperationMode(FlexRayOperationModeEnum.MAX_DATA);

		//Test of the set and get mode functions
		flexray.setMode(modeOne);
		assertEquals(modeOne, flexray.getMode());

		flexray.setMode(modeTwo);
		assertEquals(modeTwo,flexray.getMode());
	}

	/**
	 * test the Map<Integer, List<BusMessage>> insertNewMessages(Map<Integer, List<BusMessage>> messagesByControllerId,
	 * 			List<BusMessage> newMessages) function
	 */
	@Test
	public void testInsertNewMessage() {

		FlexRay flexray = new FlexRay();

		//example Data
		Map<Integer, List<BusMessage>> messagesByControllerId = new HashMap<Integer, List<BusMessage>>();
		List<BusMessage> messagesIn = new ArrayList<BusMessage>();
		BusMessage alreadyInOne = new BusMessage("in", "in".length(),NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, 1, 1);
		BusMessage notInYet = new BusMessage("not yet", "not yet".length(), PLANNED_TRAJECTORY_X, 1, 2);
		messagesIn.add(alreadyInOne);
		messagesByControllerId.put(alreadyInOne.getControllerID(), messagesIn);

		//expected Data
		Map<Integer, List<BusMessage>> finalMessagesByControllerId = new HashMap<Integer, List<BusMessage>>();
		List<BusMessage> comingMessages = new ArrayList<BusMessage>();
		comingMessages.add(notInYet);
		finalMessagesByControllerId.put(alreadyInOne.getControllerID(), messagesIn);
		finalMessagesByControllerId.put(notInYet.getControllerID(), comingMessages);

		//tests the insertion
		assertEquals(finalMessagesByControllerId, flexray.insertNewMessages(messagesByControllerId, comingMessages));
	}

	/**
	 * Tests the public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) function
	 */
	@Test
	public void testDidExectueLoop() {

	}
	
	@Test
	public void testSlotSize() {
		FlexRay flexRay = new FlexRay();
		flexRay.setMode(new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY));
		int expected = (int)Math.ceil((262 * 8 * 1000)/(double)10);
		assertEquals(expected, flexRay.getSlotSize());
		
		flexRay.setMode(new FlexRayOperationMode(FlexRayOperationModeEnum.MAX_DATA));
		expected = (int)Math.ceil((262 * 8 * 1000)/(double)20);
		assertEquals(expected, flexRay.getSlotSize());
	}
	
	/*@Test
	public void testCycleTime() {
		List<Object> components = new ArrayList<Object>();
		ModelicaPhysicalVehicle car = new ModelicaPhysicalVehicle();
		CameraSensor cam = new CameraSensor(car);
		CompassSensor com = new CompassSensor(car);
		SpeedSensor speed = new SpeedSensor(car);
		components.add(cam);
		components.add(com);													//TODO does not function
		components.add(speed);
		FlexRay flexRay = new FlexRay();
		long expected = flexRay.getSlotSize() * (3 + 4);
		assertEquals(expected, flexRay.getCycleTime());
		
	}*/
}
