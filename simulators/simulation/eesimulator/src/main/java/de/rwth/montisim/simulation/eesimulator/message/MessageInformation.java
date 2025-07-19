/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

public class MessageInformation {
    public final int msgId; // The ID for all the messages with this name.
    public final String name;
    public final DataType type;
    public final EEComponent sender;
    public int priority;

    public MessageInformation(int msgId, String name, DataType type, EEComponent sender) {
        this.msgId = msgId;
        this.name = name;
        this.type = type;
        this.sender = sender;
        this.priority = 0;
    }

}