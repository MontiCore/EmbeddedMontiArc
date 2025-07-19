/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

public class SynthesizedComponentSymbol extends EMAComponentInstanceSymbol {
    public static final SynthesizedComponentKind SYNTHESIZED_COMPONENT_KIND;

    static {
        SYNTHESIZED_COMPONENT_KIND = SynthesizedComponentKind.INSTANCE;
    }

    public SynthesizedComponentSymbol(String name, EMAComponentSymbolReference componentType) {
        super(name, componentType);
        setKind(SYNTHESIZED_COMPONENT_KIND);
    }
}
