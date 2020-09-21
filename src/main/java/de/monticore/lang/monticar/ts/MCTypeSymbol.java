/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.monticore.symboltable.types.TypeSymbol;

import java.util.List;
import java.util.Optional;

/**
 */
public interface MCTypeSymbol extends TypeSymbol, ScopeSpanningSymbol {

    MCTypeSymbolKind KIND = new MCTypeSymbolKind();

    List<? extends MCTypeSymbol> getFormalTypeParameters();

    Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass();

    List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces();

    List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes();

    boolean isFormalTypeParameter();
}
