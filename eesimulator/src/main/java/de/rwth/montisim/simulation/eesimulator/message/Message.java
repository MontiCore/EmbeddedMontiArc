/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.message;

/**
 * Messages that are transmitted over buses between EEComponent(e.g. Sensors, Actuators)
 */
public class Message  {

    /** Message ID allocated in the MessageTypeManager. */
    public int msgId;

	/** Message to be transmitted. */
	public Object message;

	/** Message length in bytes. */
	public int msgLen;

    /** Component ID from the ComponentManager */
    public int senderId;

    public Message(MessageInformation info, Object message, int msgLen, int senderId){
        this.msgId = info.messageId;
        this.message = message;
        this.msgLen = msgLen;
        this.senderId = senderId;
    }
    public Message(MessageInformation info, Object message, int senderId){
        this.msgId = info.messageId;
        this.message = message;
        this.msgLen = info.type.getDataSize();
        this.senderId = senderId;
    }
}

