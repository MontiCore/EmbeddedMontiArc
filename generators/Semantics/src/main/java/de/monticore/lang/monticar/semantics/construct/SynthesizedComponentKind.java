/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceKind;
import de.monticore.symboltable.SymbolKind;

public class SynthesizedComponentKind extends EMAComponentInstanceKind {
    private static final String NAME = "de.monticore.lang.monticar.semantics.construct.SynthesizedComponentKind";
    public static final SynthesizedComponentKind INSTANCE = new SynthesizedComponentKind();

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public boolean isKindOf(SymbolKind kind) {
        return NAME.equals(kind.getName()) || super.isKindOf(kind);
    }
}
