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
import java.time.Instant;
import java.util.UUID;

import commons.simulation.DiscreteEvent;

public class KeepAliveEvent implements DiscreteEvent{
	
	private String ID;
	
	private Bus bus;
	
	private Instant eventTime;
	
	public KeepAliveEvent(Bus bus, Instant eventTime) {
		this.eventTime = eventTime;
		this.bus = bus;
		this.ID = UUID.randomUUID().toString();
	}
	
	public Bus getBus() {
		return this.bus;
	}
	
	@Override
	public Instant getEventTime() {
		return eventTime;
	}

	@Override
	public String getEventId() {
		return ID;
	}

}
