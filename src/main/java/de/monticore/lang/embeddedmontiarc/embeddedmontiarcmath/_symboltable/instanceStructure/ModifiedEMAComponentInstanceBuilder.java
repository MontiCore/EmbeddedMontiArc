/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathExpression;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.Collection;
import java.util.Map;
import java.util.Optional;

public class ModifiedEMAComponentInstanceBuilder extends EMADynamicComponentInstanceBuilder {

    @Override
    public EMAComponentInstanceSymbol build() {
        EMAComponentInstanceSymbol instance = super.build();
        EMAComponentSymbol component = instance.getComponentType().getReferencedSymbol();



        return instance;
    }

    @Override
    protected void addOtherToComponentInstance(EMADynamicComponentInstanceSymbol instanceSymbol) {
        EMAComponentSymbol component = instanceSymbol.getComponentType().getReferencedSymbol();

        Optional<MathStatementsSymbol> math = component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND);
        if(math.isPresent()){
            instanceSymbol.getSpannedScope().getAsMutableScope().add(math.get());
        }

        Collection<MathExpressionSymbol> eprs = component.getSpannedScope().resolveLocally(MathExpressionSymbol.KIND);


        for(MathExpressionSymbol msym : eprs) {
            instanceSymbol.getSpannedScope().getAsMutableScope().add(msym);
        }

//        System.out.println(eprs);
    }

    @Override
    protected void exchangeGenerics(EMAComponentInstanceSymbol inst, Map<MCTypeSymbol, ActualTypeArgument> mapTypeArguments) {
        super.exchangeGenerics(inst, mapTypeArguments);

        Collection<MathExpressionSymbol> eprs = inst.getSpannedScope().resolveLocally(MathExpressionSymbol.KIND);

        mapTypeArguments.forEach((k, v) -> {
            eprs.forEach(mexp -> {
                if(mexp instanceof MathValueSymbol){
                    MathValueSymbol mvs = (MathValueSymbol) mexp;
                    if(mvs.getType().getType().getName().equals(k.getName())){
                        mvs.getType().getType().setName(v.getType().getName());
                    }
                }
            });
        });
    }
}
