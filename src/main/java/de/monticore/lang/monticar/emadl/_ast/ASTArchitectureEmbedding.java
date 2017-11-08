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
package de.monticore.lang.monticar.emadl._ast;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;


public class ASTArchitectureEmbedding extends ASTArchitectureEmbeddingTOP {

    private CNNArchCompilationUnitSymbol architectureSymbol;
    private PortArraySymbol inputSymbol;
    private PortArraySymbol outputSymbol;


    public ASTArchitectureEmbedding() {
    }

    public ASTArchitectureEmbedding(String inputPort, String architectureCU, String outputPort) {
        super(inputPort, architectureCU, outputPort);
    }


    public CNNArchCompilationUnitSymbol getArchitectureSymbol() {
        if (architectureSymbol == null) {
            Optional<CNNArchCompilationUnitSymbol> optSymbol = getEnclosingScope().get().resolve(getArchitectureSymbolName(), CNNArchCompilationUnitSymbol.KIND);
            if (optSymbol.isPresent()){
                architectureSymbol = optSymbol.get();
            }
            else {
                Log.error("Neural network architecture with name " + getArchitectureSymbolName() + " could not be found in the model path.");
            }
        }
        return architectureSymbol;
    }

    public PortArraySymbol getInputSymbol() {
        if (inputSymbol == null) {
            Optional<PortArraySymbol> optSymbol = getEnclosingScope().get().resolve(getInputSymbolName(), PortArraySymbol.KIND);
            if (optSymbol.isPresent()){
                inputSymbol = optSymbol.get();
            }
            else {
                Log.error("Network input port variable " + getInputSymbolName() + " does not exist.");
            }
        }
        return inputSymbol;
    }

    public PortArraySymbol getOutputSymbol() {
        if (outputSymbol == null) {
            Optional<PortArraySymbol> optSymbol = getEnclosingScope().get().resolve(getOutputSymbolName(), PortArraySymbol.KIND);
            if (optSymbol.isPresent()){
                outputSymbol = optSymbol.get();
            }
            else {
                Log.error("Network output port variable " + getOutputSymbolName() + " does not exist.");
            }
        }
        return outputSymbol;
    }
}
