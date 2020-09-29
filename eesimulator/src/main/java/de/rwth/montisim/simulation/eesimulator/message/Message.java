/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.Optional;

import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationContext;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.EESystem.EESerializationContext;
import de.rwth.montisim.simulation.eesimulator.components.BusUser;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

/**
 * Messages that are transmitted over buses between EEComponent(e.g. Sensors,
 * Actuators)
 */
public class Message implements CustomJson {

    /** Component ID from the ComponentManager */
    public BusUser sender;

    /** MessageInformation allocated in the MessageTypeManager. */
    public MessageInformation msgInfo;

    /** Message to be transmitted. */
    public Object message;

    /** Message length in bytes. */
    public int msgLen = -1;

    public Message(BusUser sender, MessageInformation info, Object message, int msgLen) {
        this.sender = sender;
        this.msgInfo = info;
        this.message = message;
        this.msgLen = msgLen;
    }

    public Message(BusUser sender, MessageInformation info, Object message) {
        this.sender = sender;
        this.msgInfo = info;
        this.message = message;
        this.msgLen = info.type.getDataSize(message);
    }

    protected Message() {
    }

    // Check for INSTANCE EQUALITY (assumes all MessageInformation shared in the MessageTypeManager)
    public boolean isMsg(MessageInformation info) {
        return info == this.msgInfo;
    }

    public static final String K_SENDER = "sender";
    public static final String K_NAME = "name";
    public static final String K_MSG = "message";
    public static final String K_LENGTH = "length";

    @Override
    public void write(JsonWriter w, SerializationContext context) throws SerializationException {
        w.startObject();
        w.write(K_SENDER, sender.properties.name);
        w.write(K_NAME, msgInfo.name);
        w.writeKey(K_MSG);
        msgInfo.type.toJson(w, message, context);
        w.write(K_LENGTH, msgLen);
        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, SerializationContext context) throws SerializationException {
        EESerializationContext c = (EESerializationContext)context;
        for (Entry e : t.streamObject()) {
            if (e.key.equals(K_SENDER)) {
                String name = t.getString().getJsonString();
                Optional<EEEventProcessor> r = c.cm.getComponent(name);
                if (!r.isPresent())
                    throw new ParsingException("Unknown component: " + name);
                this.sender = (BusUser)r.get();
            } else if (e.key.equals(K_NAME)) {
                String name = t.getString().getJsonString();
                Optional<MessageInformation> info = c.mtm.getMsgInfo(name);
                if (!info.isPresent())
                    throw new ParsingException("Unknown message type: " + name);
                this.msgInfo = info.get();
            } else if (e.key.equals(K_MSG)) {
                if (msgInfo == null)
                    throw new ParsingException("Message Info must be known before parsing the message");
                this.message = msgInfo.type.fromJson(t, context);
            } else if (e.key.equals(K_LENGTH)) {
                this.msgLen = (int) t.getLong();
            } else
                t.unexpected(e);
        }
    }
    
}

