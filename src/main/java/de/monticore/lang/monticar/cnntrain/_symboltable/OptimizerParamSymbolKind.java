/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.SymbolKind;

public class OptimizerParamSymbolKind implements SymbolKind {

    private static final String NAME = "de.monticore.lang.monticar.cnntrain._symboltable.OptimizerParamSymbolKind";

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
    }
}
