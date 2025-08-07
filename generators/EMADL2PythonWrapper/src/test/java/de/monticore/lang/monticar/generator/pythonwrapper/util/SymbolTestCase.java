/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;

import java.util.List;

/**
 *
 */
public class SymbolTestCase {
    private final EMAPortInstanceSymbol symbol;
    private final String actualSymbolName;
    private final EmadlType actualEmadlType;
    private final List<Integer> actualDimension;
    private final PortDirection actualPortDirection;

    public SymbolTestCase(EMAPortInstanceSymbol symbol,
                          String actualSymbolName,
                          EmadlType actualEmadlType,
                          List<Integer> actualDimension,
                          PortDirection actualPortDirection) {
        this.symbol = symbol;
        this.actualSymbolName = actualSymbolName;
        this.actualEmadlType = actualEmadlType;
        this.actualDimension = actualDimension;
        this.actualPortDirection = actualPortDirection;
    }

    public EMAPortInstanceSymbol getSymbol() {
        return symbol;
    }

    public String getActualSymbolName() {
        return actualSymbolName;
    }

    public EmadlType getActualEmadlType() {
        return actualEmadlType;
    }

    public List<Integer> getActualDimension() {
        return actualDimension;
    }

    public PortDirection getActualPortDirection() {
        return actualPortDirection;
    }
}
