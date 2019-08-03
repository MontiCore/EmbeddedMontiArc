package simulation.EESimulator;
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


import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import commons.controller.commons.BusEntry;
import simulation.bus.BusMessage;

public class TestComponent extends ImmutableEEComponent {
	
	private Set<Object> processedMessages = new HashSet<Object>();

	public TestComponent(EESimulator simulator) {
		super(simulator, EEComponentType.TEST_COMPONENT, Arrays.asList(BusEntry.values()), new HashMap<BusEntry, List<EEComponent>>());
	}
    public TestComponent(EESimulator simulator, List<BusEntry> listenTo) {
        super(simulator, EEComponentType.TEST_COMPONENT, listenTo, new HashMap<BusEntry, List<EEComponent>>());
    }

	@Override
	public void processEvent(EEDiscreteEvent event) {
		if(event.getEventType() != EEDiscreteEventTypeEnum.BUSMESSAGE) {
			throw new IllegalArgumentException("Event has to be a bus message but was " + event.getEventType());
		}
		else {
			BusMessage message = (BusMessage)event;
			if(!this.getSubscribedMessages().contains(message.getMessageID())) {
				throw new IllegalArgumentException("Message has unexpected messageId");
			}
			else{
				processedMessages.add(message.getMessage());
			}
		}
	}
	
	public Set<Object> getProcessedMessages(){
		return this.processedMessages;
	}
	
	public boolean processedMessage(Object message) {
		return this.processedMessages.contains(message);
	}

}
