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

import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class ASTTrainingEmbedding extends ASTTrainingEmbeddingTOP{

    private CNNTrainCompilationUnitSymbol trainConfigSymbol;

    public ASTTrainingEmbedding() {
    }

    public ASTTrainingEmbedding(String trainingCU) {
        super(trainingCU);
    }


    public CNNTrainCompilationUnitSymbol getTrainConfigSymbol() {
        if (trainConfigSymbol == null) {
            Optional<CNNTrainCompilationUnitSymbol> optSymbol = getEnclosingScope().get().resolve(getTrainConfigSymbolName(), CNNTrainCompilationUnitSymbol.KIND);
            if (optSymbol.isPresent()){
                trainConfigSymbol = optSymbol.get();
            }
            else {
                Log.error("0x04125q Neural network training configuration with name " + getTrainConfigSymbolName() + " could not be found in the model path.");
            }
        }
        return trainConfigSymbol;
    }

}
