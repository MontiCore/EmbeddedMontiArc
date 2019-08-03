package simulation.bus;

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
import java.util.Comparator;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;
import java.util.UUID;

import org.apache.commons.math3.exception.NullArgumentException;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EEDiscreteEventTypeEnum;

/**
 *
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de Copyright (c) 2017, Software
 * Engineering Group at RWTH Aachen, All rights reserved.
 *
 * This project is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 3.0 of the License, or (at your option)
 * any later version. This library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
public class BusMessage extends EEDiscreteEvent {

	private Object message;

	// in bytes
	private int messageLen;

	private int transmittedBytes;

	private boolean transmitted;

	private UUID controllerID;

	private BusEntry messageID;

	private Instant finishTime;

	private boolean error;
	
	private Set<UUID> seenComponentIds;

	/**
	 * Random number generator to determine a bit error
	 */
	Random bitError = new Random();

	public BusMessage(Object message, int messageLen, BusEntry messageID, Instant eventTime, UUID sourceID,
			EEComponent target) {
		super(EEDiscreteEventTypeEnum.BUSMESSAGE, eventTime, target);
		this.message = message;
		this.messageLen = messageLen;
		this.messageID = messageID;
		this.controllerID = sourceID;
		this.transmittedBytes = 0;
		this.transmitted = false;
		this.finishTime = Instant.EPOCH;
		this.error = false;
		this.seenComponentIds = new HashSet<UUID>();
	}
	
	private BusMessage(Object message, int messageLen, BusEntry messageID, Instant eventTime, UUID sourceID,
			EEComponent target, Set<UUID> seenComponentsIds) {
		super(EEDiscreteEventTypeEnum.BUSMESSAGE, eventTime, target);
		this.message = message;
		this.messageLen = messageLen;
		this.messageID = messageID;
		this.controllerID = sourceID;
		this.transmittedBytes = 0;
		this.transmitted = false;
		this.finishTime = Instant.EPOCH;
		this.error = false;
		this.seenComponentIds = seenComponentsIds;
	}

	public BusMessage(BusMessage busMessage) {
		super(EEDiscreteEventTypeEnum.BUSMESSAGE, busMessage.getEventTime(), busMessage.getTarget());
		this.message = busMessage.message;
		this.messageLen = busMessage.messageLen;
		this.transmittedBytes = busMessage.transmittedBytes;
		this.transmitted = busMessage.transmitted;
		this.controllerID = busMessage.controllerID;
		this.messageID = busMessage.messageID;
		this.finishTime = busMessage.finishTime;
		this.error = busMessage.error;
		this.bitError = busMessage.bitError;
		this.seenComponentIds = busMessage.seenComponentIds;
	}

	public Instant getFinishTime() {
		return finishTime;
	}

	public void setFinishTime(Instant finishTime) {
		this.finishTime = finishTime;
	}

	public Object getMessage() {
		return message;
	}

	public int getMessageLen() {
		return messageLen;
	}

	public UUID getControllerID() {
		return controllerID;
	}

	public BusEntry getMessageID() {
		return messageID;
	}

	public int getTransmittedBytes() {
		return transmittedBytes;
	}

	public boolean isTransmitted() {
		return transmitted;
	}

	public boolean isError() {
		return error;
	}

	public int getRemainingBytes() {
		return this.messageLen - this.transmittedBytes;
	}

	public int transmitBytes(int bytes, double bitErrorRate) {
		int res = -1;
		if (bytes >= 0) {
			if (this.messageLen - this.transmittedBytes - bytes <= 0) {
				transmitted = true;
				res = messageLen - this.transmittedBytes;
				transmittedBytes = messageLen;
			} else {
				transmittedBytes += bytes;
				res = bytes;
			}
			// bound defines the probability of a bit error (1/bound)
			int bound = Math.max(1, (int) ((1 / bitErrorRate) / (8 * res)));
			if (this.bitError.nextInt(bound) == 0) {
				this.error = true;
			}
		}
		return res;
	}

	/**
	 * Creates a new message with target as destination. Finish time of this is the
	 * event time of the forwarded message.
	 * @param target
	 * @return
	 */
	public BusMessage forwardTo(EEComponent target) {
		if(!this.transmitted) {
			throw new IllegalArgumentException("Only transmitted messages can be forwareded.");
		}
		else if(target == null){
			throw new NullArgumentException();
		}
		else {
			return new BusMessage(this.message, this.messageLen, this.messageID, this.finishTime, this.getTarget().getId(),
					target, this.seenComponentIds);
		}
	}
	
	public boolean hasTraveresed(Bus bus) {
		return !this.seenComponentIds.add(bus.getId());
	}
	
	@Override
	public String toString() {
		return super.toString() 
				+ "; Message: " + message
				+ "; Message length: " + messageLen
				+ "; Message id: " + messageID
				+ "; Sender Id: " + controllerID
				+ "; Transmitted Bytes: " + transmittedBytes
				+ "; Transmitted: " + transmitted
				+ "; Finisht Time: " + finishTime
				+ "; Error: " + error
				+ "; Seen Components: " + seenComponentIds;
	}

}

class BusMessageComparatorIdDesc implements Comparator<BusMessage> {
	// Used for sorting in descending order of
	public int compare(BusMessage a, BusMessage b) {
		return b.getMessageID().compareTo(a.getMessageID());
	}
}

class BusMessageComparatorTimeAsc implements Comparator<BusMessage> {
	// Used for sorting in ascending order of
	public int compare(BusMessage a, BusMessage b) {
		return a.getEventTime().compareTo(b.getEventTime());
	}
}
