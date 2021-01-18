/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;

import java.util.Optional;

public class EMAMSymbolicVariableSymbol extends MathExpressionSymbol {

    protected MathValueType type;

    private String name;

    private Optional<EMAPortInstanceSymbol> port = Optional.empty();

    public EMAMSymbolicVariableSymbol(String name) {
        this.name = name;
    }

    public boolean isPort() {
        return port.isPresent();
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

    public MathValueType getType() {
        return type;
    }

    public void setType(MathValueType type) {
        this.type = type;
    }

    public void setPort(EMAPortInstanceSymbol component) {
        this.port = Optional.ofNullable(component);
    }

    public Optional<EMAPortInstanceSymbol> getPort() {
        return port;
    }
}
