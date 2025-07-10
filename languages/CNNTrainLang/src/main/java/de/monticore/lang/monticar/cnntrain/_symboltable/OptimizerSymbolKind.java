/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.SymbolKind;

public class OptimizerSymbolKind implements SymbolKind {
    public static final OptimizerSymbolKind INSTANCE = new OptimizerSymbolKind();
    private static final String NAME = "de.monticore.lang.monticar.cnntrain._symboltable.OptimizerSymbolKind";

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
    }
}
