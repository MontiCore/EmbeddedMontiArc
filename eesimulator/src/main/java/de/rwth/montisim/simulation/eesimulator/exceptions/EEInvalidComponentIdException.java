/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

public class EEInvalidComponentIdException extends Exception {

    private static final long serialVersionUID = 8671926048620572410L;

    public EEInvalidComponentIdException(int componentId, int max) {
        super("Id: "+componentId+ " must be between 0 and "+max);
    }

}
