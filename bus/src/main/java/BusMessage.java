import java.util.Comparator;

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
public class BusMessage {

	private Object message;

	//in bytes
    private int messageLen;
    
    private int transmittedBytes;
    
    private boolean transmitted;
    
    //in mikro s
    private int requestTime;
    
    private int controllerID;
    
    private int messageID;
    
    public BusMessage(Object message, int messageLen, int messageID, int requestTime, int controllerID) {
    	this.message = message;
    	this.messageLen = messageLen;
    	this.messageID = messageID;
    	this.requestTime = requestTime;
    	this.controllerID = controllerID;
    	this.transmittedBytes = 0;
    	this.transmitted = false;
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

	public int getRequestTime() {
		return requestTime;
	}

	public void setRequestTime(int requestTime) {
		this.requestTime = requestTime;
	}

	public int getControllerID() {
		return controllerID;
	}

	public void setControllerID(int controllerID) {
		this.controllerID = controllerID;
	}


	public int getMessageID() {
		return messageID;
	}


	public void setMessageID(int messageID) {
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
	
	public int transmitBytes(int bytes) {
		int res = -1;
		if(bytes >= 0) {
			if(this.messageLen - this.transmittedBytes - bytes <= 0) {
				transmittedBytes = messageLen;
				transmitted = true;
				res = messageLen - this.transmittedBytes;
			}
			else {
				transmittedBytes += bytes;
				res = bytes;
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
        return a.getMessageID() - b.getMessageID(); 
    } 
} 
