/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import java.util.List;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

public class EEMessageTypeException extends Exception {
    private static final long serialVersionUID = -6968498943435246952L;
    String msgName;
    List<Pair<EEComponent, DataType>> compList;

    public EEMessageTypeException(String msgName, List<Pair<EEComponent, DataType>> compList) {
        this.msgName = msgName;
        this.compList = compList;
    }

    @Override
    public String getMessage() {
        StringBuilder str = new StringBuilder();
        str.append("Message " + msgName + " registered with different types:");
        for (Pair<EEComponent, DataType> p : compList) {
            str.append("\n    - Component '" + p.getKey().properties.name + "' with type " + p.getValue());
        }
        return str.toString();
    }
}