/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

public class EECyclicSetupException extends Exception {
    private static final long serialVersionUID = -5238172449278028294L;

    public EECyclicSetupException() {
        super("The Buses and Bridges form a cycle.");
    }
 
}