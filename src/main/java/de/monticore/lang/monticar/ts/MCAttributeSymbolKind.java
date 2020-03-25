/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.symboltable.SymbolKind;

/**
 */
public class MCAttributeSymbolKind implements SymbolKind {

    private static final String NAME = MCAttributeSymbolKind.class.getCanonicalName();

    protected MCAttributeSymbolKind() {
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
