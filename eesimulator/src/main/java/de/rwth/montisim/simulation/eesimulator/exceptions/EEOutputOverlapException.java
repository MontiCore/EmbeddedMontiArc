/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

public class EEOutputOverlapException extends Exception {
    private static final long serialVersionUID = 8089530988637131344L;
    public final String componentName;

    public EEOutputOverlapException(String componentName) {
        super(componentName);
        this.componentName = componentName;
    }
    
}