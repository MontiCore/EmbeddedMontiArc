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
package simulation.bus;

import java.time.Instant;

import simulation.EESimulator.EESimulator;

public class InstantBus extends Bus {

	private Instant currentTime = Instant.EPOCH;
	
	public InstantBus(EESimulator sim) {
		super(sim);
	}

	@Override
	public Instant getCurrentTime() {
		return currentTime;
	}

	@Override
	protected void simulateUntil(Instant endTime) {
		currentTime = endTime;
	}

	@Override
	protected Instant getNextFinishTime() {
		return currentTime;
	}

	@Override
	protected void registerMessage(BusMessage msg) {
		msg.transmitBytes(msg.getRemainingBytes(), 0.0);
		msg.setFinishTime(currentTime);
		this.registerMessageAtSimulator(msg);
	}

	@Override
	protected boolean hasMessages() {
		return false;
	}

	@Override
	public String getBusType() {
		return "instantBus";
	}
}
