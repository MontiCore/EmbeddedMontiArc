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

import commons.controller.commons.BusEntry;


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

	@Test
	public void testCycleTime() {
		FlexRay flexRay = new FlexRay();
		long expected = flexRay.getSlotSize() * 4;
		assertEquals(expected, flexRay.getCycleTime());
	}

	@Test
	public void testFillStaticSegment()	{
		FlexRay flexRay = new FlexRay();
		List<BusMessage> list1 = new ArrayList<>();
		List<BusMessage> list2 = new ArrayList<>();
		List<BusMessage> list3 = new ArrayList<>();
		Map<Integer, List<BusMessage>> map = new HashMap<>();
		int data = 6;

		//example data
		BusMessage message11 = new BusMessage(data, 10, BusEntry.ACTUATOR_BRAKE, 0, 37);
		BusMessage message12 = new BusMessage(data, 200, BusEntry.ACTUATOR_BRAKE, 0, 37);
		list1.add(message11);
		list1.add(message12);
		map.put(message11.getControllerID(), list1);

		BusMessage message21 = new BusMessage(data, 10, BusEntry.ACTUATOR_GEAR,0,39);
		BusMessage message22 = new BusMessage(data, Integer.MAX_VALUE, BusEntry.ACTUATOR_GEAR, 0, 39);
		list2.add(message21);
		list2.add(message22);
		map.put(message21.getControllerID(), list2);

		BusMessage message31 = new BusMessage(data, Integer.MAX_VALUE, BusEntry.ACTUATOR_ENGINE, 0, 36);
		BusMessage message32 = new BusMessage(data, 20, BusEntry.ACTUATOR_ENGINE, 0, 36);
		list3.add(message31);
		list3.add(message32);
		map.put(message31.getControllerID(), list3);

		//expected data
		List<BusMessage> expected = new ArrayList<>();
		expected.add(message11);
		expected.add(message12);
		expected.add(message21);

		assertEquals(expected, flexRay.fillStaticSegment(map, flexRay.getCycleTime()));
	}

	@Test
	public void testFillDynamicSegment(){
		FlexRay flexRay = new FlexRay();
		List<BusMessage> list = new ArrayList<>();
		List<BusMessage> expected = new ArrayList<>();
		int data = 6;

		//example data
		BusMessage message = new BusMessage(data, 500, BusEntry.ACTUATOR_ENGINE, 0, 36 );
		BusMessage messageTwo = new BusMessage(data, 250, BusEntry.ACTUATOR_GEAR, 0, 39);
		BusMessage messageThree = new BusMessage(data, 508, BusEntry.ACTUATOR_BRAKE, 0, 37);
		list.add(message);
		list.add(messageTwo);
		list.add(messageThree);

		//expected data
		expected.add(message);
		expected.add(messageTwo);

		assertEquals(expected,flexRay.fillDynamicSegment(list, 4*flexRay.getSlotSize()));
	}

}
