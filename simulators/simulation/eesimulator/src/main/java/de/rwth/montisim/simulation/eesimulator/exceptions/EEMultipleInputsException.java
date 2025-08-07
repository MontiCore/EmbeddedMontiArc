/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import java.util.Set;

import de.rwth.montisim.simulation.eesimulator.EEComponent;

public class EEMultipleInputsException extends Exception {
    private static final long serialVersionUID = 8089530988637131344L;
    public final String componentName;
    public final String messageName;
    public final Set<EEComponent> sendingComponents;

    public EEMultipleInputsException(EEComponent component, String messageName, Set<EEComponent> sendingComponents) {
        this.componentName = component.properties.name;
        this.messageName = messageName;
        this.sendingComponents = sendingComponents;
    }

    @Override
    public String getMessage() {
        String res = "Component \"" + componentName + "\" expects the input \"" + messageName + "\" from only one sender but receives it from:";
        for (EEComponent e : sendingComponents) {
            res += "\n  - " + e.properties.name;
        }
        return res;
    }

}