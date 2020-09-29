/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public class MessageInformation {
    //public final int messageId;
    public final String name;
    public final DataType type;
    public final EEComponent firstUser;
    public int priority;

    protected MessageInformation(String name, DataType type, EEComponent source)
            throws EEMessageTypeException {
        this.priority = 0;
        this.name = name;
        this.type = type;
        this.firstUser = source;
        //this.messageId = manager.registerMessage(this);
    }
}