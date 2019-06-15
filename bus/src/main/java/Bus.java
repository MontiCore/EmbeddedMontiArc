
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
import java.time.Duration;
import java.time.Instant;



public abstract class Bus {

	protected Instant currentTime;

	protected void processEvent(BusMessageTransmissionRequestEvent requestEvent) {
		this.simulateFor(Duration.between(currentTime, requestEvent.getEventTime()));
		currentTime = requestEvent.getEventTime();
		this.registerMessage(requestEvent.getMessage());
		this.removeKeepAlive();
		this.setKeepAlive();
	}
	
	protected void registerTransmittedEvent(BusMessage msg) {
		
	}

	abstract protected void simulateFor(Duration duration);

	abstract protected void removeKeepAlive();

	abstract protected void setKeepAlive();

	abstract protected void registerMessage(BusMessage msg);

}
