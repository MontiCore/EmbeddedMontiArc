/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import com.google.common.collect.Sets;
import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.montisim.commons.controller.interfaces.FunctionBlockInterface;
import simulation.bus.BusMessageEvent;

import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Generic wrapper for function blocks
 */
public class FunctionBlockAsEEComponent extends ImmutableEEComponent {

    FunctionBlockInterface functionBlock;

    Map<String, Object> inputs;

    Map<BusEntry, Integer> sizeByMessageId;

    Instant lastExecutionTime;

    public FunctionBlockAsEEComponent(EESimulator simulator, EEComponentType type, List<BusEntry> subscribedMessages,
                                      HashMap<BusEntry, List<EEComponent>> targetsByMessageId, HashMap<BusEntry, Integer> sizeByMessageId,
                                      FunctionBlockInterface functionBlock) {
        super(simulator, type, subscribedMessages, targetsByMessageId);
        //targetsByMessageId and sizeByMessageId contain different keys
        if (Sets.symmetricDifference(targetsByMessageId.keySet(), sizeByMessageId.keySet()).size() != 0) {
            throw new IllegalArgumentException("targetsByMessageId and sizeByMessageId must contain the same keys");
        }
        this.functionBlock = functionBlock;
        this.lastExecutionTime = this.getSimulator().getSimulationTime();
        inputs = new HashMap<>();
    }

    @Override
    public void processEvent(EEDiscreteEvent event) {
        if (event.getEventType() != EEDiscreteEventTypeEnum.BUSMESSAGE) {
            throw new IllegalArgumentException("FunctionBlockAsEEComponent expects BusMessageEvent as event. Event type was" + event.getEventType());
        }
        BusMessageEvent busMessage = (BusMessageEvent) event;
        //Save the message that arrive
        inputs.put(busMessage.getMessageID().toString(), busMessage.getMessage());
        //all inputs present
        if (inputs.size() == this.getSubscribedMessages().size()) {
            //Execute function block
            Duration timeDiff = Duration.between(lastExecutionTime, event.getEventTime());

            functionBlock.setInputs(inputs);
            inputs.clear();
            Map<String, Object> outputs = functionBlock.getOutputs();
            Map<String, BusEntry> messagesByName = new HashMap<>();
            for (BusEntry messageId : getTargetsByMessageId().keySet()) {
                messagesByName.put(messageId.toString(), messageId);
            }
            functionBlock.execute(timeDiff.toMillis());
            for (Map.Entry<String, Object> entry : outputs.entrySet()) {
                if (!messagesByName.containsKey(entry.getKey())) {
                    throw new IllegalStateException("Functionblock generated output that cannot be send. Output was: " + entry.getKey());
                }
                this.sendMessage(entry.getValue(), this.sizeByMessageId.get(entry.getKey()), messagesByName.get(entry.getKey()), event.getEventTime());
            }
        }
    }
}
