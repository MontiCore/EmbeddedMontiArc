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

import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class BeamSearchStart extends PredefinedUnrollDeclaration {

    private BeamSearchStart() {
        super(AllPredefinedLayers.BEAMSEARCH_NAME);
    }


    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, UnrollSymbol layer) {

        try {
            System.err.println("allElements: " + layer.getDeclaration().getBody().getElements().toString());
            List<ArchitectureElementSymbol> elements = new ArrayList<ArchitectureElementSymbol>();
            elements = layer.getDeclaration().getBody().getElements();
            System.err.println("LastElement: " + elements.get(elements.size()-1));
            //System.err.println("LastElement_Channels: " + elements.get(elements.size()-1).getOutputTypes().get(0).getChannels());
            for(ArchitectureElementSymbol item:elements){
                System.err.println("name2" + item.getOutputElement().get().toString());
                System.err.println("channels: " + item.getOutputTypes().get(0).getChannels().toString());
                System.err.println("name3" + item.getName());
            }
        }catch(Exception e){
            e.printStackTrace();
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(100) // TODO
                .height(1)
                .width(1)
                .elementType("0", "1")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, UnrollSymbol layer) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static BeamSearchStart create(){
        BeamSearchStart declaration = new BeamSearchStart();
        List<VariableSymbol> parameters = new ArrayList<>(Arrays.asList(
                new VariableSymbol.Builder()
                        .name(AllPredefinedLayers.BEAMSEARCH_MAX_LENGTH_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(99)
                        .build(),
                new VariableSymbol.Builder()
                        .name(AllPredefinedLayers.BEAMSEARCH_WIDTH_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
