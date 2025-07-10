/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import java.time.Instant;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.message.Message;

/**
 * Event for the completion of a message transfer.
 */
public class MessageReceiveEvent extends EEEvent {
    public static final int type = registerType(MessageReceiveEvent.class);
    public final static String TYPE_NAME = "receive";

    private Message msg;

    public MessageReceiveEvent(EEComponent target, Instant eventTime, Message msg) {
        super(target, eventTime);
        this.msg = msg;
    }

    protected MessageReceiveEvent() {
    }

    public Message getMessage() {
        return msg;
    }

    @Typed(MessageReceiveEvent.TYPE_NAME)
    public static class MessageReceiveEventData extends EventData {
        Message msg;

        MessageReceiveEventData(MessageReceiveEvent event) {
            super(event);
            msg = event.msg;
        }

        protected MessageReceiveEventData() {
        }

        @Override
        public EEEvent getEvent(EESystem eesystem) {
            return new MessageReceiveEvent(EEEvent.getTarget(target, eesystem), time, msg);
        }
    }

    @Override
    public EventData getEventData() {
        return new MessageReceiveEventData(this);
    }

    @Override
    public int getType() {
        return type;
    }
}
