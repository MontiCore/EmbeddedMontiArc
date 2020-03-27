/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

public class EEMissingComponentException  extends Exception {
    private static final long serialVersionUID = 4595621502929340728L;

    public EEMissingComponentException(String componentName) {
        super("Missing component: "+componentName);
    }

}