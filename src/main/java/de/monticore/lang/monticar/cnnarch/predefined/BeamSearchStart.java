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
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._ast.*;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.se_rwth.commons.Joiners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class BeamSearchStart extends PredefinedUnrollDeclaration {

    private BeamSearchStart() {
        super(AllPredefinedLayers.BEAMSEARCH_NAME);
    }

    List<LayerSymbol> layers = new ArrayList<>(Arrays.asList());

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, UnrollSymbol layer, VariableSymbol.Member member) {
        List<ArchTypeSymbol> output = new ArrayList<ArchTypeSymbol>();

        for(ASTArchitectureElement item: ((ASTUnroll) layer.getAstNode().get()).getBody().getElementsList()){
            if(item instanceof ASTLayer) {
                try {
                    //ArchitectureElementSymbol item = (ArchitectureElementSymbol) ASTitem;
                    LayerSymbol sublayer = (LayerSymbol) item.getSymbol();
                    sublayer.resolve();
                    System.err.println("inputElement: " + sublayer.getInputElement().get().toString());
                    System.err.println("resolved: " + sublayer.getResolvedThis().get().getInputElement().get().getOutputTypes().get(0).getChannels().toString());
                    //output = sublayer.getResolvedThis().get().getInputElement().get().getOutputTypes();
                    System.err.println("inputTypes: " + sublayer.getInputTypes().toString());
                    layers.add((LayerSymbol) sublayer);
                    System.err.println("outputTypes before: " + ((ArchitectureElementSymbol) sublayer).getOutputTypes().get(0).getChannels());
                    //System.err.println("arg0_NAME: " + ((LayerSymbol) sublayer).getArguments().get(0).getName());
                    //System.err.println("arg0_VALUE: " + ((LayerSymbol) sublayer).getIntValue((((LayerSymbol) sublayer).getArguments().get(0).getName())));
                    //item.setOutputTypes(item.getOutputTypes());
                    //System.err.println("outputTypes after: " + ((ArchitectureElementSymbol) astElement).getOutputTypes());
                } catch (Exception e) {
                    LayerSymbol sublayer = (LayerSymbol) item.getSymbol();
                    //System.err.println("The following names could not be resolved: " + Joiners.COMMA.join(sublayer.getUnresolvableVariables()));
                    e.printStackTrace();
                }
            }else if(item instanceof ASTVariable){
                try {
                    //ArchitectureElementSymbol item = (ArchitectureElementSymbol) ASTitem;
                    VariableSymbol sublayer = (VariableSymbol) item.getSymbol();
                    sublayer.resolve();
                    System.err.println("resolved2: " + sublayer.getResolvedThis().get());
                    //System.err.println("resolved2_1: " + sublayer.getResolvedThis().get().getInputElement().get().toString());
                    System.err.println("isOutput?: " + sublayer.isOutput());
                    System.err.println("Definition: " + sublayer.getIoDeclaration().toString());
                    System.err.println("Domain: " + sublayer.getIoDeclaration().getType().getDomain().toString());
                    System.err.println("Type: " + sublayer.getIoDeclaration().getType().getChannels().toString());
                    //output = sublayer.getResolvedThis().get().getInputElement().get().getOutputTypes();
                    if(sublayer.isOutput()){
                        output = new ArrayList<ArchTypeSymbol>();
                    }
                    //item.setOutputTypes(item.getOutputTypes());
                    //System.err.println("outputTypes after: " + ((ArchitectureElementSymbol) astElement).getOutputTypes());
                } catch (Exception e) {
                    VariableSymbol sublayer = (VariableSymbol) item.getSymbol();
                    //System.err.println("The following names could not be resolved2: " + Joiners.COMMA.join(sublayer.getUnresolvableVariables()));
                    e.printStackTrace();
                }
            }
        }


        System.err.println("output: " + output);
        return output;
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, UnrollSymbol layer, VariableSymbol.Member member) {
        //errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static BeamSearchStart create(){
        BeamSearchStart declaration = new BeamSearchStart();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BEAMSEARCH_MAX_LENGTH)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BEAMSEARCH_T_NAME)
                        .constraints(Constraints.INTEGER, Constraints.NON_NEGATIVE)
                        .defaultValue(0)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BEAMSEARCH_WIDTH_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
