/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.types.TypeSymbolKind;

/**
 */
public class MCTypeSymbolKind extends TypeSymbolKind {

    private static final String NAME = MCTypeSymbolKind.class.getName();

    protected MCTypeSymbolKind() {
    }

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || super.isKindOf(kind);
    }
}
