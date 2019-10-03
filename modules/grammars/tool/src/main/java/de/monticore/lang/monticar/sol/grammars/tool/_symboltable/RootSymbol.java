/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import java.util.List;

public interface RootSymbol extends AttributableSymbol {
    void addResource(ResourceSymbolReference resource);
    List<ResourceSymbolReference> getResources();
    List<ResourceSymbol> getResourceSymbols();
}
