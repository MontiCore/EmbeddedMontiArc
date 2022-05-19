/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

public class EEMissingOutputException extends Exception {
    private static final long serialVersionUID = -4953246595284389816L;

    public EEMissingOutputException(String portName, String component) {
        super("Component " + component + " requires input " + portName + ".");
    }

}