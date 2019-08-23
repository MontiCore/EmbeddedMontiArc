/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;


public class CNNTrainCompilationUnitSymbol extends CNNTrainCompilationUnitSymbolTOP{

    private ConfigurationSymbol configuration;

    public CNNTrainCompilationUnitSymbol(String name) {
        super(name);
    }

    public ConfigurationSymbol getConfiguration() {
        return configuration;
    }

    public void setConfiguration(ConfigurationSymbol configuration) {
        this.configuration = configuration;
    }
}
