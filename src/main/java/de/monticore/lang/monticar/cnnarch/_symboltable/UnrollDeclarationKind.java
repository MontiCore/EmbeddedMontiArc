/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.SymbolKind;

public class UnrollDeclarationKind implements SymbolKind {

    private static final String NAME = "de.monticore.lang.monticar.cnnarch._symboltable.UnrollDeclarationKind";

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
    }

}
