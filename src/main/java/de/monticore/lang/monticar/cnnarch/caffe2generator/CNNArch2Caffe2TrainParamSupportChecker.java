/* (c) https://github.com/MontiCore/monticore */
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
