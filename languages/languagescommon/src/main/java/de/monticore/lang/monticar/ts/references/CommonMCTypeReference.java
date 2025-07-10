/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts.references;

import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.types.references.CommonTypeReference;

/**
 * Default implementation of {@link MCTypeReference}.
 *
 */
public class CommonMCTypeReference<T extends MCTypeSymbol> extends CommonTypeReference<T> implements MCTypeReference<T> {

    public CommonMCTypeReference(String referencedSymbolName, SymbolKind referencedSymbolKind,
                                 Scope definingScopeOfReference) {

        super(referencedSymbolName, referencedSymbolKind, definingScopeOfReference);
    }
}
