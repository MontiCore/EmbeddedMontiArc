package simulation.EESimulator; /**
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

import simulation.bus.Bus;

import java.time.Instant;
import java.util.UUID;


public class KeepAliveEvent extends EEDiscreteEvent{
	
	private String ID;
	
	private Bus bus;
	

	public KeepAliveEvent(Bus bus, Instant eventTime, EEComponent target) {
		super(eventTime, target);
		this.bus = bus;
		this.ID = UUID.randomUUID().toString();
	}
	
	public Bus getBus() {
		return this.bus;
	}


	public String getEventId() {
		return ID;
	}

}
