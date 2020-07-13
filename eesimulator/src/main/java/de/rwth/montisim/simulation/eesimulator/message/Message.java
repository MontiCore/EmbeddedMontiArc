/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.lang.reflect.InvocationTargetException;
import java.util.Optional;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationContext;
import de.rwth.montisim.simulation.eesimulator.EESimulator.EESerializationContext;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

/**
 * Messages that are transmitted over buses between EEComponent(e.g. Sensors,
 * Actuators)
 */
public class Message implements CustomJson {

    /** Component ID from the ComponentManager */
    public int senderId = -1;

    /** Message ID allocated in the MessageTypeManager. */
    public int msgId = -1;

    /** Message to be transmitted. */
    public Object message;

    /** Message length in bytes. */
    public int msgLen = -1;

    public Message(int senderId, MessageInformation info, Object message, int msgLen) {
        this.senderId = senderId;
        this.msgId = info.messageId;
        this.message = message;
        this.msgLen = msgLen;
    }

    public Message(int senderId, MessageInformation info, Object message) {
        this.senderId = senderId;
        this.msgId = info.messageId;
        this.message = message;
        this.msgLen = info.type.getDataSize();
    }

    protected Message() {
    }

    public static final String K_SENDER = "sender";
    public static final String K_NAME = "name";
    public static final String K_MSG = "message";
    public static final String K_LENGTH = "length";

    public void toJson(JsonWriter j, ComponentManager cm, MessageTypeManager mtm) {
    }

    public void fromJson(JsonTraverser j, ComponentManager cm, MessageTypeManager mtm) {
    }

    @Override
    public void write(JsonWriter w, SerializationContext context) throws IllegalAccessException {
        EESerializationContext c = (EESerializationContext)context;
        w.startObject();
        w.write(K_SENDER, c.cm.componentTable.elementAt(senderId).properties.name);
        MessageInformation inf = c.mtm.messageIds.elementAt(msgId);
        w.write(K_NAME, inf.name);
        w.writeKey(K_MSG);
        inf.type.toJson(w, message, context);
        w.write(K_LENGTH, msgLen);
        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, SerializationContext context)
            throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException {
        DataType type = null;
        EESerializationContext c = (EESerializationContext)context;
        for (Entry e : t.streamObject()) {
            if (e.key.equals(K_SENDER)) {
                String name = t.getString().getJsonString();
                Optional<EEEventProcessor> r = c.cm.getComponent(name);
                if (!r.isPresent())
                    throw new ParsingException("Unknown component: " + name);
                this.senderId = r.get().id;
            } else if (e.key.equals(K_NAME)) {
                String name = t.getString().getJsonString();
                Integer i = c.mtm.messageIdByName.get(name);
                if (i == null)
                    throw new ParsingException("Unknown message type: " + name);
                this.msgId = i;
                type = c.mtm.messageIds.elementAt(i).type;
            } else if (e.key.equals(K_MSG)) {
                if (type == null)
                    throw new ParsingException("Message type must be known before parsing the message");
                this.message = type.fromJson(t, context);
            } else if (e.key.equals(K_LENGTH)) {
                this.msgLen = (int) t.getLong();
            } else
                t.unexpected(e);
        }
    }
    
}

