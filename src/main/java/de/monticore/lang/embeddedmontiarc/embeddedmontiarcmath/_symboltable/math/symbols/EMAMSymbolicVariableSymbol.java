/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

public class EMAMSymbolicVariableSymbol extends MathExpressionSymbol {

    private boolean isPort = false;

    private String name;

    public EMAMSymbolicVariableSymbol(String name) {
        this.name = name;
    }

    public boolean isPort() {
        return isPort;
    }

    public void setPort(boolean port) {
        isPort = port;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String getTextualRepresentation() {
        return getName();
    }
}
