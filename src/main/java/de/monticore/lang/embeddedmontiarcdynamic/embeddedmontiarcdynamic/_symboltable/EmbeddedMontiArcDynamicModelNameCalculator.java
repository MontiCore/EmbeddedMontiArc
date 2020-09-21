/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcModelNameCalculator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.symboltable.SymbolKind;

import java.util.LinkedHashSet;
import java.util.Set;

public class EmbeddedMontiArcDynamicModelNameCalculator extends EmbeddedMontiArcModelNameCalculator {

    @Override
    public Set<String> calculateModelNames(String name, SymbolKind kind) {
        Set<String> calculatedModelNames = super.calculateModelNames(name, kind);
        if(EMADynamicComponentInstanceSymbol.KIND.isKindOf(kind)){
            calculatedModelNames.addAll(this.calculateModelNameForEMAComponentInstance(name));
        }
        return calculatedModelNames;
    }
}
