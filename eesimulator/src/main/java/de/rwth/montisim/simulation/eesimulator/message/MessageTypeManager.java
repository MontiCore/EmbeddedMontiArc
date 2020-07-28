/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

/**
 * Handles out message IDs for each message (by name).
 * The type has to be unique for a given message name across the simulation.
 */
public class MessageTypeManager {
    public HashMap<String, HashSet<EEMessageTypeException>> messageTypeErrors = new HashMap<>();
    int idCounter = 0;
    HashMap<String, MessageInformation> messages = new HashMap<>();

    public int registerMessage(MessageInformation info) throws EEMessageTypeException {        
        if (messages.containsKey(info.name)) {
            MessageInformation i = messages.get(info.name);
            if (!i.type.equals(info.type)) {
                throw new EEMessageTypeException(
                    info.name,
                    i.firstUser, i.type,
                    info.firstUser, info.type
                );
            }
            return i.messageId;
        }
        int id = idCounter++;
        messages.put(info.name, info);
        return id;
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