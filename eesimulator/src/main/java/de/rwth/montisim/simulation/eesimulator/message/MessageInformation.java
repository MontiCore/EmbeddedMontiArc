/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.message;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public class MessageInformation {
    public final int messageId;
    public final String name;
    public final DataType type;
    public final EEComponent firstUser;
    public int priority;

    public MessageInformation(String name, DataType type, MessageTypeManager manager, EEComponent source)
            throws EEMessageTypeException {
        this.priority = 0;
        this.name = name;
        this.type = type;
        this.firstUser = source;
        this.messageId = manager.registerMessage(this);
    }
}