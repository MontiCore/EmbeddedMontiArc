/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public class EEComponentTypeException extends Exception {
    private static final long serialVersionUID = -6666072759009657158L;

    public EEComponentTypeException(String name, EEComponentType type, EEComponentType expected) {
        super("Component " + name + " is of type: " + type + " but expected type: " + expected);
    }
 
}