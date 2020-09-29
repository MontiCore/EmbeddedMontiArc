/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

/**
 * Super class for all exceptions occurring during the setup of the EEVehicle
 */
public class EESetupException extends Exception {
    private static final long serialVersionUID = 8202462683854071829L;
    public final EESetupErrors errors;

    public EESetupException(EESetupErrors errors) {
        super(errors.toString());
        this.errors = errors;
    }

}