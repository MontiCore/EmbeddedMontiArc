/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

/**
 * Messages that are transmitted over buses between EEComponent(e.g. Sensors, Actuators)
 */
public class Message  {

    /** Component ID from the ComponentManager */
    public int senderId;
    
    /** Message ID allocated in the MessageTypeManager. */
    public int msgId;

	/** Message to be transmitted. */
	public Object message;

	/** Message length in bytes. */
	public int msgLen;


    public Message(int senderId, MessageInformation info, Object message, int msgLen){
        this.senderId = senderId;
        this.msgId = info.messageId;
        this.message = message;
        this.msgLen = msgLen;
    }
    public Message(int senderId, MessageInformation info, Object message){
        this.senderId = senderId;
        this.msgId = info.messageId;
        this.message = message;
        this.msgLen = info.type.getDataSize();
    }
}

