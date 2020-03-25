/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.Symbol;

/**
 *
 */
public interface MCFieldSymbol extends Symbol {

    MCAttributeSymbolKind KIND = new MCAttributeSymbolKind();

    MCTypeReference<? extends MCTypeSymbol> getType();

    boolean isParameter();
}
