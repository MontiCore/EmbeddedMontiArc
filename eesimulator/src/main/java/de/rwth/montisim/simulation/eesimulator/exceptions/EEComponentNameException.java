/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

public class EEComponentNameException extends Exception {
    private static final long serialVersionUID = -470880630607615494L;

    public EEComponentNameException(String name) {
        super("Multiple components with name: " + name);
    }
 
}