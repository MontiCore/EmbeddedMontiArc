/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import java.util.List;

public interface AttributableSymbol {
    void addAttribute(AttributeSymbolReference attribute);
    List<AttributeSymbolReference> getAttributes();
    List<AttributeSymbol> getAttributeSymbols();
}
