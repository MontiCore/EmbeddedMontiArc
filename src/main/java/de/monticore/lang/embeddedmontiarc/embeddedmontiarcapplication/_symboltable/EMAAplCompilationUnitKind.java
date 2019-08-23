/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._symboltable;

import de.monticore.symboltable.SymbolKind;

public class EMAAplCompilationUnitKind implements SymbolKind {

    private static final String NAME = "EMAAplCompilationUnitKind";

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
    }

}
