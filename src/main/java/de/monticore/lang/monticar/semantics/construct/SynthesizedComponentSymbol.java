/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolReference;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;

public class SynthesizedComponentSymbol extends EMADynamicComponentInstantiationSymbol {
    public static final SynthesizedComponentKind SYNTHESIZED_COMPONENT_KIND;

    static {
        SYNTHESIZED_COMPONENT_KIND = SynthesizedComponentKind.INSTANCE;
    }

    public SynthesizedComponentSymbol(String name, EMADynamicComponentSymbolReference componentType) {
        super(name, componentType);
        setKind(SYNTHESIZED_COMPONENT_KIND);
    }

    public SynthesizedComponentSymbol(String name, EMADynamicComponentSymbolReference componentType, String dimensionDependsOnName) {
        super(name, componentType, dimensionDependsOnName);
        setKind(SYNTHESIZED_COMPONENT_KIND);
    }

    public SynthesizedComponentSymbol(String name, EMADynamicComponentSymbolReference componentType, String nonDynamicDimensionDependsOnName, String dimensionDependsOnName) {
        super(name, componentType, nonDynamicDimensionDependsOnName, dimensionDependsOnName);
        setKind(SYNTHESIZED_COMPONENT_KIND);
    }
}
