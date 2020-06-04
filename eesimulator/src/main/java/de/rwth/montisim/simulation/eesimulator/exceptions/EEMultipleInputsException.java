/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.exceptions;

import java.util.List;

public class EEMultipleInputsException extends Exception {
    private static final long serialVersionUID = 8089530988637131344L;
    public final String componentName;
    public final String messageName;
    public final List<String> sendingComponents;

    public EEMultipleInputsException(String componentName, String messageName, List<String> sendingComponents) {
        this.componentName = componentName;
        this.messageName = messageName;
        this.sendingComponents = sendingComponents;
    }
    
    @Override
    public String getMessage(){
        String res = "Component \"" + componentName + "\" expects the input \"" + messageName + "\" from only one sender but receives it from:";
        for (String s : sendingComponents){
            res += "\n  - " + s;
        }
        return res;
    }

}