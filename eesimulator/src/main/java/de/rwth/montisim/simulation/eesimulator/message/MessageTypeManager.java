/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

/**
 * Handles out message IDs for each message (by name).
 * The type has to be unique for a given message name across the simulation.
 */
public class MessageTypeManager {
    public HashMap<String, HashSet<EEMessageTypeException>> messageTypeErrors = new HashMap<>();
    HashMap<String, MessageInformation> messages = new HashMap<>();

    public MessageInformation registerMessage(String name, DataType type, EEComponent source) throws EEMessageTypeException {        
        if (messages.containsKey(name)) {
            MessageInformation i = messages.get(name);
            if (!i.type.equals(type)) {
                throw new EEMessageTypeException(
                    name,
                    i.firstUser, i.type,
                    source, type
                );
            }
            return i;
        }
        MessageInformation i = new MessageInformation(name, type, source);
        messages.put(name, i);
        return i;
    }

    public Optional<MessageInformation> getMsgInfo(String msgName){
        MessageInformation info = messages.get(msgName);
        if(info == null) return Optional.empty();
        return Optional.of(info);
    }

    /** 
     * Sets the priority of all <b>registered</b> messages contained in the priorities list.
     * The lower the number, the higher the priority.
    */
    public void addMessagePriorities(List<Pair<String,Integer>> priorities) {
        for (Pair<String, Integer> p : priorities){
            MessageInformation info = messages.get(p.getKey());
            if (info != null) info.priority = p.getValue();
        }
    }

}