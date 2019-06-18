import java.util.Comparator;
import java.util.Random;

import commons.controller.commons.BusEntry;

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
public class BusMessage extends DiscreteEvent{

	private Object message;

	//in bytes
    private int messageLen;
    
    private int transmittedBytes;
    
    private boolean transmitted;
    
    /**
     * Time the request to transmit the messages was placed. (in microseconds)
     */
    //private long requestTime;
    
    private int controllerID;
    
    private BusEntry messageID;
    
    private long finishTime;
    
    private boolean error;

	/**
	 * -----------Added Type, Target and Priority--------------
	 * Changed constructor and getters/setters
	 */
	//private MessageType type;

    //private EEComponent target;

    private int priority;



    
	/**
	 * Random number generator to determine a bit error
	 */
	Random bitError = new Random();
    
    public BusMessage(Object message, int messageLen, BusEntry messageID, int requestTime, int controllerID, int priority, MessageType type, EEComponent target) {
    	super(type, target, requestTime);
    	this.message = message;
    	this.messageLen = messageLen;
    	this.messageID = messageID;
    	this.controllerID = controllerID;
    	this.priority = priority;
    	this.transmittedBytes = 0;
    	this.transmitted = false;
    	this.finishTime = -1l;
    	this.error = false;
    }
    

    public int getPriority(){return priority;}

	//public MessageType getType() { return type;	}

	//public void setType(MessageType type) { this.type = type;	}

	//public EEComponent getTarget() { return target;	}

	public long getFinishTime() {
		return finishTime;
	}

	public void setFinishTime(long finishTime) {
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

	//public long getRequestTime() { return requestTime;	}

	//public void setRequestTime(long requestTime) { this.requestTime = requestTime;	}

	public int getControllerID() {
		return controllerID;
	}

	public void setControllerID(int controllerID) {
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
		if(bytes >= 0) {
			if(this.messageLen - this.transmittedBytes - bytes <= 0) {
				transmitted = true;
				res = messageLen - this.transmittedBytes;
				transmittedBytes = messageLen;
			}
			else {
				transmittedBytes += bytes;
				res = bytes;
			}
			//bound defines the probability of a bit error (1/bound)
			int bound = Math.max(1, (int)((1/bitErrorRate)/(8*res)));
			if(this.bitError.nextInt(bound) == 0) {
				this.error = true;
			}
		}
		return res;
	}
}

class BusMessageComparator implements Comparator<BusMessage> 
{ 
    // Used for sorting in ascending order of 
    public int compare(BusMessage a, BusMessage b) 
    { 
        return a.getMessageID().compareTo( b.getMessageID()); 
    } 
} 
