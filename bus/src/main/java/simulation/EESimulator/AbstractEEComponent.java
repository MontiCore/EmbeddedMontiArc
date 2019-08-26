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
package simulation.EESimulator;

import java.time.Instant;
import java.util.Collections;
import java.util.List;
import java.util.UUID;

import org.apache.commons.math3.exception.NullArgumentException;

import commons.controller.commons.BusEntry;
import simulation.bus.BusMessage;

abstract class AbstractEEComponent implements EEComponent{

	private final EESimulator simulator;
	
	private final EEComponentType componentType;
	
	private final UUID Id;
	
	public AbstractEEComponent(EESimulator simulator, EEComponentType type) {
		if (simulator == null || type == null) {
			throw new NullArgumentException();
		}
		this.simulator = simulator;
		this.componentType = type;
		this.Id = UUID.randomUUID();
	}
	
	@Override
	public EEComponentType getComponentType() {
		return componentType;
	}
	
	@Override
	public EESimulator getSimulator() {
		return simulator;
	}

	@Override
	public UUID getId() {
		return Id;
	}
	
	/**
	 * Send message to all targets that are registered for this message in targetsByMessageId
	 * @param message message to be send
	 * @param messageLen length of message in bytes
	 * @param messageId message id of the message
	 * @param eventTime time when the message is sent
	 */
	@Override
	public void sendMessage(Object message, int messageLen , BusEntry messageId, Instant eventTime) {
		List<EEComponent> targets = this.getTargetsByMessageId().getOrDefault(messageId, Collections.emptyList());
		for(EEComponent target : targets) {
			BusMessage msg = new BusMessage(message, 6, messageId,
					eventTime, this.getId(), target);
			this.getSimulator().addEvent(msg);
		}
	}
}
