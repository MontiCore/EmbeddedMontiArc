/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Vector;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

/**
 * Handles out message IDs for each message (by name).
 * The type has to be unique for a given message name across the simulation.
 */
public class MessageTypeManager {
    public HashMap<String, HashSet<EEMessageTypeException>> messageTypeErrors = new HashMap<>();
    Vector<MessageInformation> messageIds = new Vector<>();
    HashMap<String, Integer> messageIdByName = new HashMap<>();

    public int registerMessage(MessageInformation info) throws EEMessageTypeException {
        if (messageIdByName.containsKey(info.name)) {
            int id = messageIdByName.get(info.name);
            MessageInformation i = messageIds.get(id);
            if (!i.type.equals(info.type)) {
                throw new EEMessageTypeException(
                    info.name,
                    i.firstUser, i.type,
                    info.firstUser, info.type
                );
            }
            return id;
        }
        int id = messageIds.size();
        messageIds.add(info);
        messageIdByName.put(info.name, id);
        return id;
    }

    public MessageInformation getMsgInfo(int msgId){
        return messageIds.elementAt(msgId);
    }

    /** 
     * Sets the priority of all <b>registered</b> messages contained in the priorities list.
     * The lower the number, the higher the priority.
    */
    public void addMessagePriorities(List<Pair<String,Integer>> priorities) {
        for (Pair<String, Integer> p : priorities){
            if (messageIdByName.containsKey(p.getKey())){
                messageIds.get(messageIdByName.get(p.getKey())).priority = p.getValue();
            }
        }
    }

}