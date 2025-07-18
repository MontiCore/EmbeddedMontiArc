/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.symboltable.SymbolKind;

/**
 *
 */
public class MCMethodSymbolKind implements SymbolKind {

    private static final String NAME = MCMethodSymbolKind.class.getCanonicalName();

    protected MCMethodSymbolKind() {
    }

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
    }
}
