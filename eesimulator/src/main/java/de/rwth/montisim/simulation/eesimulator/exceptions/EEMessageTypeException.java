/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;

public class EEMessageTypeException extends Exception {
    private static final long serialVersionUID = -6968498943435246952L;

    public EEMessageTypeException(String name, EEComponent c1, DataType type1, EEComponent c2, DataType type2) {
        super ("Message " + name + " registered with different types:\n\tComponent '"+c1.name+ "' with type "+type1+".\nComponent '"+c2.name+"' with type "+type2+".");
	}
}