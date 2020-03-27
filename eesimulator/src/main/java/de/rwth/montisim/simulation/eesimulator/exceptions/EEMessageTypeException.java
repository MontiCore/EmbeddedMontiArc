/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

public class EEMessageTypeException extends Exception {
    private static final long serialVersionUID = -6968498943435246952L;
    private final EEComponent comp;
    private final DataType type;

    public EEMessageTypeException(EEComponent comp, DataType type) {
        super("Component '"+comp.name+"' registered with type '"+type+"'.");
        this.comp = comp;
        this.type = type;
    }
    @Override
    public boolean equals(Object obj){
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        EEMessageTypeException e = (EEMessageTypeException) obj;
        return this.comp == e.comp && this.type.equals(e.type);
    }
    @Override
    public int hashCode() {
        return comp.hashCode() ^ type.hashCode();
    }
}