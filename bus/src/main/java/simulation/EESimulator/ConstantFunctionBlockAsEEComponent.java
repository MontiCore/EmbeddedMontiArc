/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import com.google.common.collect.Sets;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.FunctionBlockInterface;
import simulation.bus.Bus;

import java.util.*;

/**
 * Wrapper for a function block that always sends the messages.
 */
public class ConstantFunctionBlockAsEEComponent extends ImmutableEEComponent{

    /**
     * BusEntries that are send by this component by their name.
     */
    private Map<String, BusEntry> busEntryByName = new HashMap<>();

    /**
     * Size of the message by BusEntry
     */
    private final HashMap<BusEntry, Integer> sizeByMessageId;

    /**
     * Create a constant function block as EEComponent with default settings.
     * @param buses Buses that the EEComponent should be connected
     * @param sizeByMessageId Size of the messages that are transmitted
     * @param functionBlock The function block that is wrapped.
     * @return ConstantFunctionBlockAsEEComponent
     */
    public static ConstantFunctionBlockAsEEComponent createConstantFunctionBlockAsEEComponent(List<Bus> buses, HashMap<BusEntry, Integer> sizeByMessageId, FunctionBlockInterface functionBlock){
        if(buses == null || buses.isEmpty()){
            throw new IllegalArgumentException("Buses can not be null or empty");
        }
        Set<String> outputMessageNames = functionBlock.getOutputs().keySet();
        List<EEComponent> targets = new ArrayList<>(buses);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        for(BusEntry entry : BusEntry.values()){
            if(outputMessageNames.contains(entry.toString())){
                targetsByMessageId.put(entry, targets);
            }
        }
        return new ConstantFunctionBlockAsEEComponent(buses.get(0).getSimulator(), targetsByMessageId, sizeByMessageId, functionBlock);
    }

    /**
     * Create a constant function block as EEComponent with default settings.
     * @param bus Bus that the EEComponent should be connected
     * @param sizeByMessageId Size of the messages that are transmitted
     * @param functionBlock The function block that is wrapped.
     * @return ConstantFunctionBlockAsEEComponent
     */
    public  static ConstantFunctionBlockAsEEComponent createConstantFunctionBlockAsEEComponent(Bus bus, HashMap<BusEntry, Integer> sizeByMessageId, FunctionBlockInterface functionBlock){
        return createConstantFunctionBlockAsEEComponent(Collections.singletonList(bus), sizeByMessageId, functionBlock);
    }

    public ConstantFunctionBlockAsEEComponent(EESimulator simulator, HashMap<BusEntry, List<EEComponent>> targetsByMessageId, HashMap<BusEntry, Integer> sizeByMessageId, FunctionBlockInterface functionBlock) {
        super(simulator, EEComponentType.FUNCTION_BLOCK, Collections.emptyList(), targetsByMessageId);
        if(Sets.symmetricDifference(targetsByMessageId.keySet(), sizeByMessageId.keySet()).size() != 0){
            throw new IllegalArgumentException("targetsByMessageId and sizeByMessageId have to have the same keys.");
        }
        for(BusEntry entry : targetsByMessageId.keySet()){
            busEntryByName.put(entry.toString(), entry);
        }
        this.sizeByMessageId = sizeByMessageId;
        this.setConstantOutput(functionBlock.getOutputs());
    }

    /**
     * Send the constant messages by the wrapped function block
     * @param outputs Messages that should be send. Indexed by the names of the message Ids.
     */
    private void setConstantOutput(Map<String, Object> outputs) {
        for(Map.Entry<String, Object> entry : outputs.entrySet()){
            BusEntry messageId = busEntryByName.get(entry.getKey());
            this.sendMessage(entry.getValue(), sizeByMessageId.get(messageId), messageId, getSimulator().getSimulationTime());
        }
    }

    @Override
    public void processEvent(EEDiscreteEvent event) {
        throw new UnsupportedOperationException("Constant function block does not expect any event. Event was: " + event);
    }
}
