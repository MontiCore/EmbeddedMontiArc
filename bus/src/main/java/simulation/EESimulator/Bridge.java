/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.time.Duration;
import org.apache.commons.lang3.tuple.Pair;

/**
 * Used to connect to buses.
 */
public class Bridge extends MutableEEComponent{
    /**
     * The two buses that should be connected
     */
    protected Pair<Bus, Bus> connected;

    /**
     * The delay that is added to each event if it is transmitted over this bride.
     */
    private final Duration delay;

    public Bridge (Pair<Bus, Bus> connected, Duration delay){
        super(connected.getLeft().getSimulator(), EEComponentType.BRIDGE);
        if(connected.getLeft().getSimulator() != connected.getRight().getSimulator()) {
            throw new IllegalArgumentException("Bus with different simulators can not be connected");
        }
        this.connected = connected;
        this.delay = delay;

        //initialize subscribed messages and targetsByMessageId
        Set<BusEntry> subscribedMessages = new HashSet<BusEntry>(connected.getKey().getSubscribedMessages());
        subscribedMessages.addAll(connected.getValue().getSubscribedMessages());
        this.subscribedMessages.addAll(subscribedMessages);
        for(BusEntry messageId : subscribedMessages) {
            List<EEComponent> targets = new ArrayList<EEComponent>();
            if(connected.getKey().getSubscribedMessages().contains(messageId)) {
                targets.add(connected.getKey());
            }
            if(connected.getValue().getSubscribedMessages().contains(messageId)) {
                targets.add(connected.getValue());
            }
            this.targetsByMessageId.put(messageId, targets);
        }

        connected.getKey().registerComponent(this);
        connected.getValue().registerComponent(this);
    }

    @Override
    public void processEvent(EEDiscreteEvent event){
        if(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE){
            BusMessage msg = (BusMessage) event;
            msg.setFinishTime(msg.getEventTime().plus(delay));
            msg.transmitBytes(msg.getMessageLen(), 0);
            if(msg.getControllerID() == connected.getLeft().getId()) {
                this.getSimulator().addEvent(msg.forwardTo(connected.getRight()));
            }
            else if(msg.getControllerID() == connected.getRight().getId()) {
                this.getSimulator().addEvent(msg.forwardTo(connected.getLeft()));
            }
            else {
                throw new IllegalArgumentException("Message from invalid controller" + msg.getControllerID() + "received at: " + this.toString());
            }
        }
        else{
            throw new IllegalArgumentException("Only BusMessages events expected at " + this.toString() + " but was: " + event.getEventType());
        }
    }

    /**
     * Updates the subscribed messages of this and the other connected buses.
     * Add bus as target for messages ids.
     * @param bus Bus which needs the messages with messageIds
     * @param messageIds Ids of messages that bus wants to receive.
     */
    public void update(Bus bus, List<BusEntry> messageIds){
        for(BusEntry messageId : messageIds) {
            List<EEComponent> targets = this.targetsByMessageId.getOrDefault(messageId, new ArrayList<EEComponent>());
            targets.add(bus);
            this.targetsByMessageId.put(messageId, targets);
            if(!subscribedMessages.contains(messageId)) {
                this.subscribedMessages.add(messageId);
            }
        }
        if(bus.equals(connected.getKey())){
            connected.getValue().addSubscribedMessages(this, messageIds);
        }
        else{
            if(bus.equals(connected.getValue())){
                connected.getKey().addSubscribedMessages(this, messageIds);
            }
            else{
                throw new IllegalArgumentException("Message send by unknown Bus.");
            }
        }
    }

    public Pair<Bus, Bus> getConnectedBuses() {
        return this.connected;
    }

    public Duration getDelay() {
        return delay;
    }
}
