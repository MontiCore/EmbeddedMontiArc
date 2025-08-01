/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.Comparator;

import de.rwth.montisim.commons.utils.BuildObject;
import de.rwth.montisim.simulation.eesimulator.EESystem;

/**
 * Used for sorting in descending order of message priority
 */
public class MessagePriorityComparator implements Comparator<Message>, BuildObject {
    public static final String CONTEXT_KEY = "msg_prio_comparator";
    final EESystem eesystem;

    public MessagePriorityComparator(EESystem eesystem) {
        this.eesystem = eesystem;
    }

    public int compare(Message a, Message b) {
        int r = Integer.compare(
                a.msgInfo.priority,
                b.msgInfo.priority
        );
        if (r != 0) return r;
        return Integer.compare(
                eesystem.getComponentPriority(b.msgInfo.sender.id),
                eesystem.getComponentPriority(a.msgInfo.sender.id)
        );
    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }
}