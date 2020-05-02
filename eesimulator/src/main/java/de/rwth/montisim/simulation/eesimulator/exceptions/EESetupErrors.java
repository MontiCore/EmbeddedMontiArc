/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * Super class for all exceptions occurring during the setup of the EEVehicle
 */
public class EESetupErrors {

    public Optional<EECyclicSetupException> cyclicError = Optional.empty();
    public List<EEComponentNameException> namesErrors = new ArrayList<>();
    public HashMap<String, HashSet<EEMessageTypeException>> messageTypeErrors = new HashMap<>();
    public List<EEMissingComponentException> missingComponentExceptions = new ArrayList<>();
    public List<EEComponentTypeException> componentTypeExceptions = new ArrayList<>();
    public List<EEInvalidComponentIdException> invalidIdExceptions = new ArrayList<>();
    public List<EEMultipleInputsException> multipleInputsExceptions = new ArrayList<>();
    public List<EEMissingOutputException> missingOutputExceptions = new ArrayList<>();

    public void throwExceptions() throws EESetupException {
        if (
            cyclicError.isPresent() || 
            !namesErrors.isEmpty() || 
            !messageTypeErrors.isEmpty() || 
            !missingComponentExceptions.isEmpty() ||
            !componentTypeExceptions.isEmpty() ||
            !invalidIdExceptions.isEmpty() ||
            !multipleInputsExceptions.isEmpty() ||
            !missingOutputExceptions.isEmpty()
        )
            throw new EESetupException(this);
    }

    @Override
    public String toString() {
        String res = "\n# EE Setup Errors\n";
        if (cyclicError.isPresent()){
            res += "## EECyclicSetupException: " +cyclicError.get().getMessage() + '\n';
        }
        res += printExceptions("EEComponentNameException", namesErrors);
        res += printExceptions("EEMessageTypeException","sent with different types", messageTypeErrors);
        res += printExceptions("EEMissingComponentException", missingComponentExceptions);
        res += printExceptions("EEComponentTypeException", componentTypeExceptions);
        res += printExceptions("EEInvalidComponentIdException", invalidIdExceptions);
        res += printExceptions("EEMultipleInputsException", multipleInputsExceptions);
        res += printExceptions("EEMissingOutputException", missingOutputExceptions);
        return res;
    }

    private static String printExceptions(String name, String reason, HashMap<String, ? extends Set<? extends Exception>> exceptions) {
        String res = "";
        if (!exceptions.isEmpty()){
            res += "## "+name+ "s:\n";
            for (HashMap.Entry<String, ? extends Set<? extends Exception>> e : exceptions.entrySet()) {
                res += "- Message '" + e.getKey() + "' "+reason+":\n";
                for (Exception c : e.getValue()){
                    res += "    - " + c.getMessage() + '\n';
                }
            }
        }
        return res;
    }

    private static String printExceptions(String name, List<? extends Exception> exceptions) {
        String res = "";
        if (!exceptions.isEmpty()){
            res += "## "+name+ "s:\n";
            for (Exception e : exceptions){
                res += "- " + e.getMessage() + '\n';
            }
        }
        return res;
    }

    public void addMessageTypeError(String msgName, EEMessageTypeException e1, EEMessageTypeException e2){
        HashSet<EEMessageTypeException> comps = messageTypeErrors.get(msgName);
        if (comps == null) {
            comps = new HashSet<>();
            messageTypeErrors.put(msgName, comps);
        }
        comps.add(e1);
        comps.add(e2);
    }
}