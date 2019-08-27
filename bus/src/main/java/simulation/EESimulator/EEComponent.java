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
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

import commons.controller.commons.BusEntry;

public interface EEComponent {
	
	public EEComponentType getComponentType();

	public List<BusEntry> getSubscribedMessages();
	
	public HashMap<BusEntry, List<EEComponent>> getTargetsByMessageId();
	
	public EESimulator getSimulator() ;

	/**
	 * processes an incoming event
	 */
	public void processEvent(EEDiscreteEvent event);

	/**
	 *
	 * @return Randomly generated ID of this component
	 */
	public UUID getId();
	
	
	public void sendMessage(Object message, int messageLen , BusEntry messageId, Instant eventTime);
}
