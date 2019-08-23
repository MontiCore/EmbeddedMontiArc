/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.symboltable.Symbol;

import java.util.Collection;

/**
 *
 */
public class SymbolTestCaseLibrary {
    public SymbolTestCase PRIMITIVE_Q_SYMBOL;
    public SymbolTestCase VECTOR_Q_SYMBOL;
    public SymbolTestCase MATRIX_Q_SYMBOL;
    public SymbolTestCase CUBE_Q_SYMBOL;
    public SymbolTestCase PRIMITIVE_Z_SYMBOL;
    public SymbolTestCase VECTOR_Z_SYMBOL;
    public SymbolTestCase MATRIX_Z_SYMBOL;
    public SymbolTestCase CUBE_Z_SYMBOL;
    public SymbolTestCase ONE_PLACE_VECTOR_Z_SYMBOL;
    public SymbolTestCase PRIMITIVE_B_SYMBOL;
    public SymbolTestCase OUTPUT_PRIMITIVE_Z_SYMBOL;

    private final EMAComponentInstanceSymbol allTypesInstance;

    SymbolTestCaseLibrary(final EMAComponentInstanceSymbol allTypesInstance) {
        this.allTypesInstance = allTypesInstance;
    }

    void loadSymbols() {
        Collection<Collection<Symbol>> allSymbols = allTypesInstance.getSpannedScope().getLocalSymbols().values();
        for (Collection<Symbol> symbolCollection : allSymbols) {
            for (Symbol symbol : symbolCollection) {
                if (symbol instanceof EMAPortInstanceSymbol) {
                    EMAPortInstanceSymbol portInstanceSymbol = (EMAPortInstanceSymbol)symbol;
                    final String symbolName = portInstanceSymbol.getName();
                    switch (symbolName) {
                        case "q1":
                            PRIMITIVE_Q_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Q,
                                    Lists.newArrayList(1), PortDirection.INPUT);
                            break;
                        case "q2":
                            VECTOR_Q_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Q,
                                    Lists.newArrayList(4), PortDirection.INPUT);
                            break;
                        case "q3":
                            MATRIX_Q_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Q,
                                    Lists.newArrayList(3, 6), PortDirection.INPUT);
                            break;
                        case "q4":
                            CUBE_Q_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Q,
                                    Lists.newArrayList(2, 3, 5), PortDirection.INPUT);
                            break;
                        case "z1":
                            PRIMITIVE_Z_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Z,
                                    Lists.newArrayList(1), PortDirection.INPUT);
                            break;
                        case "z2":
                            VECTOR_Z_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Z,
                                    Lists.newArrayList(6), PortDirection.INPUT);
                            break;
                        case "z3":
                            MATRIX_Z_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Z,
                                    Lists.newArrayList(7, 3), PortDirection.INPUT);
                            break;
                        case "z4":
                            CUBE_Z_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Z,
                                    Lists.newArrayList(2, 2, 7), PortDirection.INPUT);
                            break;
                        case "z5":
                            ONE_PLACE_VECTOR_Z_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Z,
                                    Lists.newArrayList(1), PortDirection.INPUT);
                            break;
                        case "b1":
                            PRIMITIVE_B_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.B,
                                    Lists.newArrayList(1), PortDirection.INPUT);
                            break;
                        case "result":
                            OUTPUT_PRIMITIVE_Z_SYMBOL = new SymbolTestCase(portInstanceSymbol, symbolName, EmadlType.Z,
                                    Lists.newArrayList(1), PortDirection.OUTPUT);
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}
