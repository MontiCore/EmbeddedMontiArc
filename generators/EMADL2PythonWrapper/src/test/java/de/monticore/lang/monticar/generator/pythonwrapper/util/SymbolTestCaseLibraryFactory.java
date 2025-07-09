/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

/**
 *
 */
public class SymbolTestCaseLibraryFactory {
    private static SymbolTestCaseLibrary testCaseLibrary;
    private final static String MODEL_COMPONENT_IDENTIFIER = "all-types";

    public SymbolTestCaseLibraryFactory() {

    }

    public SymbolTestCaseLibrary getSymbolTestCaseLibrary() {
        if (testCaseLibrary == null) {
            ComponentInstanceSymbolLibraryFactory componentInstanceFactory = new ComponentInstanceSymbolLibraryFactory();
            ComponentInstanceSymbolLibrary library = componentInstanceFactory.getComponentInstanceLibrary();

            testCaseLibrary = new SymbolTestCaseLibrary(library.getModelByIdentifier(MODEL_COMPONENT_IDENTIFIER));
            try {
                testCaseLibrary.loadSymbols();
            } catch (Exception e) {
                testCaseLibrary = null;
                throw new IllegalStateException("Could not load library!");
            }
        }
        return testCaseLibrary;
    }
}
