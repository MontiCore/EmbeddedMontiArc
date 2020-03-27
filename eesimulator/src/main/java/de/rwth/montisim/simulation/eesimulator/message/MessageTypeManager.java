/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupErrors;

/**
 * Handles out message IDs for each message (by name).
 * The type has to be unique for a given message name across the simulation.
 */
public class MessageTypeManager {
    protected final EESetupErrors errors;
    public final MessagePriorityComparator msgPrioComp;
    Vector<MessageInformation> messageIds = new Vector<>();
    HashMap<String, Integer> messageIdByName = new HashMap<>();
    
    public MessageTypeManager(ComponentManager compManager, EESetupErrors errors){
        this.errors = errors;
        this.msgPrioComp = new MessagePriorityComparator(this, compManager);
    }

    public int registerMessage(MessageInformation info) {
        if (messageIdByName.containsKey(info.name)) {
            int id = messageIdByName.get(info.name);
            MessageInformation i = messageIds.get(id);
            if (!i.type.equals(info.type)) {
                errors.addMessageTypeError(
                    info.name,
                    new EEMessageTypeException(i.firstUser, i.type),
                    new EEMessageTypeException(info.firstUser, info.type)
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