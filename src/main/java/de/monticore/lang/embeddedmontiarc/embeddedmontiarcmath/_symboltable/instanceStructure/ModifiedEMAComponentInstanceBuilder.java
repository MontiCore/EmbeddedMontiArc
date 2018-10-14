package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;

import java.util.Optional;

public class ModifiedEMAComponentInstanceBuilder extends EMADynamicComponentInstanceBuilder {

    @Override
    public EMAComponentInstanceSymbol build() {
        EMAComponentInstanceSymbol instance = super.build();
        EMAComponentSymbol component = instance.getComponentType().getReferencedSymbol();

        Optional<MathStatementsSymbol> math = component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND);
        if(math.isPresent()){
            instance.getSpannedScope().getAsMutableScope().add(math.get());
        }

        return instance;
    }
}
