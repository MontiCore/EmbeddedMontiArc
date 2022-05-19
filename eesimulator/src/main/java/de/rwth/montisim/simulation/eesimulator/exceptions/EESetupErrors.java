/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import java.util.ArrayList;
import java.util.List;

/**
 * Super class for all exceptions occurring during the setup of the EEVehicle
 */
public class EESetupErrors {

    public List<EEMessageTypeException> msgTypeExceptions = new ArrayList<>();
    public List<EEComponentNameException> namesErrors = new ArrayList<>();
    public List<EEMissingComponentException> missingComponentExceptions = new ArrayList<>();
    public List<EEMultipleInputsException> multipleInputsExceptions = new ArrayList<>();
    public List<EEMissingOutputException> missingOutputExceptions = new ArrayList<>();

    public void throwExceptions() throws EESetupException {
        if (
                !msgTypeExceptions.isEmpty() ||
                        !namesErrors.isEmpty() ||
                        !missingComponentExceptions.isEmpty() ||
                        !multipleInputsExceptions.isEmpty() ||
                        !missingOutputExceptions.isEmpty()
        )
            throw new EESetupException(this);
    }

    @Override
    public String toString() {
        String res = "\n# EE Setup Errors\n";
        res += printExceptions("EEMessageTypeException", msgTypeExceptions);
        res += printExceptions("EEComponentNameException", namesErrors);
        res += printExceptions("EEMissingComponentException", missingComponentExceptions);
        res += printExceptions("EEMultipleInputsException", multipleInputsExceptions);
        res += printExceptions("EEMissingOutputException", missingOutputExceptions);
        return res;
    }


    private static String printExceptions(String name, List<? extends Exception> exceptions) {
        String res = "";
        if (!exceptions.isEmpty()) {
            res += "## " + name + "s:\n";
            for (Exception e : exceptions) {
                res += "- " + e.getMessage() + '\n';
            }
        }
        return res;
    }


}