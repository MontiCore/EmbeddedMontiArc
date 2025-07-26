/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.Optional;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;

/**
 * Messages that are transmitted over buses between EEComponent(e.g. Sensors,
 * Actuators)
 */
public class Message implements CustomJson {

    /**
     * Static information associated to this message.
     */
    public MessageInformation msgInfo;

    /**
     * Message to be transmitted.
     */
    public Object message;

    /**
     * Message length in bytes.
     */
    public int msgLen = -1;

    public Message(MessageInformation info, Object message, int msgLen) {
        this.msgInfo = info;
        this.message = message;
        this.msgLen = msgLen;
    }

    public Message(MessageInformation info, Object message) {
        this.msgInfo = info;
        this.message = message;
        this.msgLen = info.type.getDataSize(message);
    }

    protected Message() {
    }

    public boolean isMsg(int msgId) {
        return this.msgInfo.msgId == msgId;
    }

    public static final String K_SENDER = "sender";
    public static final String K_NAME = "name";
    public static final String K_MSG = "message";
    public static final String K_LENGTH = "length";

    @Override
    public void write(JsonWriter w, BuildContext context) throws SerializationException {
        w.startObject();
        w.write(K_SENDER, msgInfo.sender.properties.name);
        w.write(K_NAME, msgInfo.name);
        w.writeKey(K_MSG);
        msgInfo.type.toJson(w, message, context);
        w.write(K_LENGTH, msgLen);
        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, BuildContext context) throws SerializationException {
        EESystem eesystem = context.getObject(EESystem.CONTEXT_KEY);
        EEComponent sender = null;
        for (Entry e : t.streamObject()) {
            if (e.key.equals(K_SENDER)) {
                String name = t.getString().getJsonString();
                Optional<EEComponent> r = eesystem.getComponent(name);
                if (!r.isPresent())
                    throw new ParsingException("Unknown component: " + name);
                sender = (EEComponent) r.get();
            } else if (e.key.equals(K_NAME)) {
                if (sender == null)
                    throw new ParsingException("Sender must be known before parsing the MessageInformation");
                String name = t.getString().getJsonString();
                this.msgInfo = sender.getMsgInfo(name);
                if (this.msgInfo == null)
                    throw new ParsingException("Unknown message type: " + name);
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

