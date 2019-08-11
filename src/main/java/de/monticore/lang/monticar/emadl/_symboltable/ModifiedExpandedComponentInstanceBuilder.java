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
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterType;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class ModifiedExpandedComponentInstanceBuilder extends EMAComponentInstanceBuilder {

    @Override
    public EMAComponentInstanceSymbol build() {
        EMAComponentInstanceSymbol instance = super.build();
        EMAComponentSymbol component = instance.getComponentType().getReferencedSymbol();
        Optional<ArchitectureSymbol> architecture = component.getSpannedScope()
                .resolve("", ArchitectureSymbol.KIND);

        if (architecture.isPresent()){

            addPortArraySymbolsToInstance(instance);
            addParameterSymbolsToInstance(instance);

            ArchitectureSymbol architectureInstance = architecture.get()
                    .preResolveDeepCopy(instance.getSpannedScope());
            instance.getSpannedScope().getAsMutableScope().add(architectureInstance);
        }
        return instance;
    }

    public void addParameterSymbolsToInstance(EMAComponentInstanceSymbol instance){
        //add generics
        for (ResolutionDeclarationSymbol sym : instance.getResolutionDeclarationSymbols()){
            if (sym.getASTResolution() instanceof ASTUnitNumberResolution){
                ASTUnitNumberResolution numberResolution = (ASTUnitNumberResolution) sym.getASTResolution();
                ParameterSymbol genericsParam = new ParameterSymbol.Builder()
                        .name(sym.getNameToResolve())
                        .type(ParameterType.ARCHITECTURE_PARAMETER)
                        .build();
                genericsParam.setExpression(ArchSimpleExpressionSymbol.of(numberResolution.getNumber().get()));
                instance.getSpannedScope().getAsMutableScope().add(genericsParam);
            }
            else {
                Log.error("Argument type error. Arguments of a CNN component " +
                                "that are not numbers are currently not supported."
                        , sym.getSourcePosition());
            }
        }

        //add configuration parameters
        for (int i = 0; i < instance.getArguments().size(); i++){
            if (instance.getArguments().get(i) instanceof ASTNumberExpression){
                ASTNumberExpression exp = (ASTNumberExpression) instance.getArguments().get(i);

                MCFieldSymbol emaParam = instance.getComponentType().getConfigParameters().get(i);
                ParameterSymbol archParam = new ParameterSymbol.Builder()
                        .name(emaParam.getName())
                        .type(ParameterType.ARCHITECTURE_PARAMETER)
                        .build();
                archParam.setExpression(ArchSimpleExpressionSymbol.of(
                        exp.getNumberWithUnit().getNumber().get()));

                instance.getSpannedScope().getAsMutableScope().add(archParam);
            }
            else {
                Log.error("Argument type error. Arguments of a CNN component " +
                                "that are not numbers are currently not supported."
                        , instance.getArguments().get(i).get_SourcePositionStart());
            }
        }
    }

    public void addPortArraySymbolsToInstance(EMAComponentInstanceSymbol instance){
        Map<String, List<EMAPortInstanceSymbol>> nameToPortList = new HashMap<>();
        for (EMAPortInstanceSymbol port : instance.getPortInstanceList()){
            List<EMAPortInstanceSymbol> list = nameToPortList
                    .computeIfAbsent(port.getNameWithoutArrayBracketPart(), k -> new ArrayList<>());
            list.add(port);
        }

        for (String name : nameToPortList.keySet()){
            if (!instance.getSpannedScope().resolveLocally(name, EMAPortArraySymbol.KIND).isPresent()) {
                List<EMAPortInstanceSymbol> ports = nameToPortList.get(name);
                EMAPortArraySymbol portArray = new EMAPortArraySymbol(name, null);
                portArray.setDimension(ports.size());
                portArray.setDirection(ports.get(0).isIncoming());
                portArray.setTypeReference(ports.get(0).getTypeReference());
                instance.getSpannedScope().getAsMutableScope().add(portArray);
            }
        }
    }

}
