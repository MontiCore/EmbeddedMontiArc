/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
