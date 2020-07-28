/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.Comparator;

import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;

/**
 * Used for sorting in descending order of message priority
 */
public class MessagePriorityComparator implements Comparator<Message> {
    final MessageTypeManager msgManager;
    final ComponentManager compManager;
    public MessagePriorityComparator(MessageTypeManager msgManager, ComponentManager compManager){
        this.msgManager = msgManager;
        this.compManager = compManager;
    }

    public int compare(Message a, Message b) {
        int r = Integer.compare(
            a.msgInfo.priority,
            b.msgInfo.priority
        );
        if (r != 0) return r;
        return Integer.compare(
            compManager.getComponentPriority(b.sender.id),
            compManager.getComponentPriority(a.sender.id)
        );
    }
}