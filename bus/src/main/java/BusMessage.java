import java.time.Instant;
import java.util.Comparator;
import java.util.Random;

import commons.controller.commons.BusEntry;
import commons.simulation.*;

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
public class BusMessage implements DiscreteEvent {

	private Object message;

	// in bytes
	private int messageLen;

	private int transmittedBytes;

	private boolean transmitted;

	/**
	 * Time the request to transmit the messages was placed.
	 */
	private Instant requestTime;

	private String controllerID;

	private BusEntry messageID;

	private Instant finishTime;

	private boolean error;

	private MessageType type;

	private EEComponent target;

	/**
	 * Random number generator to determine a bit error
	 */
	Random bitError = new Random();

	public BusMessage(Object message, int messageLen, BusEntry messageID, Instant requestTime, MessageType type,
			EEComponent target) {
		this.message = message;
		this.messageLen = messageLen;
		this.messageID = messageID;
		this.controllerID = this.messageID.toString();
		this.transmittedBytes = 0;
		this.transmitted = false;
		this.finishTime = Instant.EPOCH;
		this.error = false;
	}

	public Instant getFinishTime() {
		return finishTime;
	}

	public MessageType getType() {
		return type;
	}

	public void setType(MessageType type) {
		this.type = type;
	}

	public EEComponent getTarget() {
		return target;
	}

	public void setFinishTime(Instant finishTime) {
		this.finishTime = finishTime;
	}

	public Object getMessage() {
		return message;
	}

	public void setMessage(Object message) {
		this.message = message;
	}

	public int getMessageLen() {
		return messageLen;
	}

	public void setMessageLen(int messageLen) {
		this.messageLen = messageLen;
	}

	public Instant getRequestTime() {
		return requestTime;
	}

	public void setRequestTime(Instant requestTime) {
		this.requestTime = requestTime;
	}

	public String getControllerID() {
		return controllerID;
	}

	public void setControllerID(String controllerID) {
		this.controllerID = controllerID;
	}

	public BusEntry getMessageID() {
		return messageID;
	}

	public void setMessageID(BusEntry messageID) {
		this.messageID = messageID;
	}

	public int getTransmittedBytes() {
		return transmittedBytes;
	}

	public void setTransmittedBytes(int transmittedBytes) {
		this.transmittedBytes = transmittedBytes;
	}

	public boolean isTransmitted() {
		return transmitted;
	}

	public boolean isError() {
		return error;
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

	@Override
	public Instant getEventTime() {
		Instant res = Instant.EPOCH;
		if (type == MessageType.SEND) {
			return this.requestTime;
		} else if (type == MessageType.RECEIVE) {
			return this.finishTime;
		}
		return res;
	}

	@Override
	public String getEventId() {
		return this.messageID.toString();
	}

	public void forwardToBus(String controllerID) {
		this.requestTime = this.finishTime;
		this.finishTime = Instant.EPOCH;
		this.transmitted = false;
		this.error = false;
		this.transmittedBytes = 0;
		this.controllerID = controllerID;
	}
}

class BusMessageComparatorIdAsc implements Comparator<BusMessage> {
	// Used for sorting in ascending order of
	public int compare(BusMessage a, BusMessage b) {
		return a.getMessageID().compareTo(b.getMessageID());
	}
}

class BusMessageComparatorTimeAsc implements Comparator<BusMessage> {
	// Used for sorting in ascending order of
	public int compare(BusMessage a, BusMessage b) {
		return a.getRequestTime().compareTo(b.getRequestTime());
	}
}
