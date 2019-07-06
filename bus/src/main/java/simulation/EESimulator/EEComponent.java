/**
 *
 *  ******************************************************************************
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
package simulation.EESimulator;

import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Set;
import java.util.UUID;

import commons.controller.commons.BusEntry;

import java.util.ArrayList;
import java.util.List;


public abstract class EEComponent {
	
	protected EESimulator simulator;
	protected EEComponentType componentType;
	protected HashMap<BusEntry, List<EEComponent>> sendTo; //which message to which component
	protected List<BusEntry> listenTo; //which message do I want to get
	
	protected EEComponent(EESimulator simulator) {
		if(simulator == null) {
			throw new IllegalArgumentException("simulator can't be null");
		}
		this.simulator = simulator;
		sendTo = new HashMap<BusEntry, List<EEComponent>>();
		listenTo = new ArrayList<BusEntry>();
	}

	public EEComponentType getComponentType() {
		return componentType;
	}

	public List<BusEntry> getListenTo() {
		return listenTo;
	}

	/**
     * processes an incoming event
     */
    public abstract void processEvent(EEDiscreteEvent event);

    /**
     *
     * @return Randomly generated ID of this component
     */
    public abstract UUID getID();
}
