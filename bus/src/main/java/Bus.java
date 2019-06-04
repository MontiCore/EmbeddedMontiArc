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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import commons.controller.commons.BusEntry;
import commons.simulation.DiscreteEvent;
import commons.simulation.DiscreteEventSimulationNotifiable;
import commons.simulation.DiscreteEventSimulator;

public abstract class Bus extends DiscreteEventSimulator{
	
	protected Map<String, BusMessage> deliveredMessages = new HashMap<String, BusMessage>();
	
	protected List<BusMessage> activeMessages = new ArrayList<BusMessage>();
	
	protected int connectedComponents = 0;
	
	@Override
    public void registerDiscreteEventSimulationNotifiable(DiscreteEventSimulationNotifiable simulationNotifiable) {
		connectedComponents++;
		super.registerDiscreteEventSimulationNotifiable(simulationNotifiable);
    }
	
	@Override
    public void unregisterDiscreteEventSimulationNotifiable(DiscreteEventSimulationNotifiable simulationNotifiable) {
        this.connectedComponents--;
		super.unregisterDiscreteEventSimulationNotifiable(simulationNotifiable);
    }
	
	@Override
	protected void processEvent(DiscreteEvent event) {
		if(event instanceof BusMessageDeliveredEvent) {
			BusMessageDeliveredEvent deliveredEvent = (BusMessageDeliveredEvent)event; 
			deliveredMessages.put(deliveredEvent.getMessage().getMessageID().toString(), deliveredEvent.getMessage());
		}
		else if(event instanceof BusMessageTransmissionRequestEvent) {
			BusMessageTransmissionRequestEvent requestEvent = (BusMessageTransmissionRequestEvent)event;
			activeMessages.add(requestEvent.getMessage());
		}
		else {
			throw new IllegalArgumentException("Event of wrong type. Expected a bus event but was a " + event.getClass().toString());
		}
	}
	
    public Optional<BusMessage> getData(String key) {
		return Optional.ofNullable(this.deliveredMessages.get(key));
	}

    public Map<String, BusMessage> getDeliveredMessages(){
    	return this.deliveredMessages;
    }

    public Set<String> getImportNames() { 
    	 return this.deliveredMessages.keySet();
    }
}
