/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.symboltable.SymbolKind;

public class EMADynamicComponentSymbolKIND implements SymbolKind {
    private static final String NAME = "de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolKIND";

    public EMADynamicComponentSymbolKIND() {
    }

    public String getName() {
        return "de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolKIND";
    }

    public boolean isKindOf(SymbolKind kind) {
        return "de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolKIND".equals(kind.getName()) ;
    }
}
