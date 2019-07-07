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
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;
import de.monticore.lang.monticar.cnntrain._ast.*;

public class CNNArch2Caffe2TrainParamSupportChecker extends TrainParamSupportChecker {

    public void visit(ASTLoadCheckpointEntry node){
        printUnsupportedEntryParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTNormalizeEntry node){
        printUnsupportedEntryParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTNesterovOptimizer node){
        printUnsupportedOptimizer(node.getName());
        this.unsupportedElemList.add(this.unsupportedOptFlag);
    }

    public void visit(ASTAdaDeltaOptimizer node){
        printUnsupportedOptimizer(node.getName());
        this.unsupportedElemList.add(this.unsupportedOptFlag);
    }

    public void visit(ASTMinimumLearningRateEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTRescaleGradEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTClipGradEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTGamma2Entry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTCenteredEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTClipWeightsEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

}