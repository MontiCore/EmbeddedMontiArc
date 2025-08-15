/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.controller.interfaces;

import java.util.Map;

/**
 * This interface provides the basic structures of a function block
 *
 */
public interface FunctionBlockInterface {

    /***************************************************************************
     * Main function for this function block *
     ***************************************************************************/
    /**
     * main method
     */
    public abstract void execute(double timeDelta);

    /***************************************************************************
     * Getter and Setter to emulate the Input and Output of a function block *
     ***************************************************************************/
    /**
     * set connectionMap from extern
     *
     * @param inputs map that contains all connectionMap with in getImportNames
     *               specified keys
     */
    public void setInputs(Map<String, Object> inputs);

    /**
     * output method
     *
     * @return all outputs in a map
     */
    public abstract Map<String, Object> getOutputs();

    /**
     * returns all import names for maps
     *
     * @return all import names
     */
    public abstract String[] getImportNames();
}
