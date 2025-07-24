/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;

import java.util.List;

import static com.google.common.base.Preconditions.checkArgument;

/**
 *
 */
class EmadlInstanceSymbol2PortVariableMapper {
    private final EmadlInstanceSymbolUtil symbolUtil;

    EmadlInstanceSymbol2PortVariableMapper(EmadlInstanceSymbolUtil emadlInstanceSymbolUtil) {
        this.symbolUtil = emadlInstanceSymbolUtil;
    }

    public PortVariable map(EMAPortInstanceSymbol symbol) {
        checkArgument(willAccept(symbol), "EMAPortInstanceSymbol is not supported");
        final String variableName = symbolUtil.retrieveSymbolName(symbol);
        final PortDirection portDirection = symbolUtil.retrievePortType(symbol);
        final List<Integer> dimension = symbolUtil.retrieveDimensions(symbol);
        final EmadlType emadlType = symbolUtil.retrieveEmadlType(symbol);

        PortVariable portVariable;
        if (symbolUtil.isPrimitiveType(symbol)) {
            portVariable = PortVariable.primitiveVariableFrom(variableName, emadlType, portDirection);
        } else {
            portVariable = PortVariable.multidimensionalVariableFrom(variableName, emadlType, portDirection, dimension);
        }

        return portVariable;
    }

    public boolean willAccept(EMAPortInstanceSymbol symbol) {
        return symbol != null && symbolUtil.willAccept(symbol);
    }
}
