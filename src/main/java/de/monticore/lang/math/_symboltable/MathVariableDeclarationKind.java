/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.SymbolKind;

/**
 */
public class MathVariableDeclarationKind implements SymbolKind {

    private static final String NAME = "MathVariableDeclarationKind";

    /**
     * get the name of this kind of variable
     *
     * @return name as String
     */
    @Override
    public String getName() {
        return NAME;
    }

    /**
     * checks if its a kind of the symbol
     *
     * @return TRUE, if its a kind, otherwise FALSE
     */
    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
    }
}
