/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.building;

public class PythonModuleBuildingException extends Exception {
    public PythonModuleBuildingException(String s) {
        super("Building of python module failed: " + s);
    }
}
