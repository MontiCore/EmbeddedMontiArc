package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain._visitor.CNNTrainVisitor;
import de.se_rwth.commons.logging.Log;
import java.util.ArrayList;
import java.util.List;

public class TrainParamSupportChecker implements CNNTrainVisitor {

    private List<String> unsupportedElemList = new ArrayList();

    private void printUnsupportedEntryParam(String nodeName){
        Log.warn("Unsupported training parameter " + "'" + nodeName + "'" + " for the backend CAFFE2. It will be ignored.");
    }

    private void printUnsupportedOptimizer(String nodeName){
        Log.warn("Unsupported optimizer parameter " + "'" + nodeName + "'" + " for the backend CAFFE2. It will be ignored.");
    }

    private void printUnsupportedOptimizerParam(String nodeName){
        Log.warn("Unsupported training optimizer parameter " + "'" + nodeName + "'" + " for the backend CAFFE2. It will be ignored.");
    }

    public TrainParamSupportChecker() {
    }

    public static final String unsupportedOptFlag = "unsupported_optimizer";

    public List getUnsupportedElemList(){
        return this.unsupportedElemList;
    }

    //Empty visit method denotes that the corresponding training parameter is supported.
    //To set a training parameter as unsupported, add the corresponding node to the unsupportedElemList
    public void visit(ASTNumEpochEntry node){}

    public void visit(ASTBatchSizeEntry node){}

    public void visit(ASTLoadCheckpointEntry node){
        printUnsupportedEntryParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTNormalizeEntry node){
        printUnsupportedEntryParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTTrainContextEntry node){}

    public void visit(ASTEvalMetricEntry node){}

    public void visit(ASTSGDOptimizer node){}

    public void visit(ASTAdamOptimizer node){}

    public void visit(ASTRmsPropOptimizer node){}

    public void visit(ASTAdaGradOptimizer node){}

    public void visit(ASTNesterovOptimizer node){
        printUnsupportedOptimizer(node.getName());
        this.unsupportedElemList.add(this.unsupportedOptFlag);
    }

    public void visit(ASTAdaDeltaOptimizer node){
        printUnsupportedOptimizer(node.getName());
        this.unsupportedElemList.add(this.unsupportedOptFlag);
    }

    public void visit(ASTLearningRateEntry node){}

    public void visit(ASTMinimumLearningRateEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTWeightDecayEntry node){}

    public void visit(ASTLRDecayEntry node){}

    public void visit(ASTLRPolicyEntry node){}

    public void visit(ASTRescaleGradEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTClipGradEntry node){
        printUnsupportedOptimizerParam(node.getName());
        this.unsupportedElemList.add(node.getName());
    }

    public void visit(ASTStepSizeEntry node){}

    public void visit(ASTMomentumEntry node){}

    public void visit(ASTBeta1Entry node){}

    public void visit(ASTBeta2Entry node){}

    public void visit(ASTEpsilonEntry node){}

    public void visit(ASTGamma1Entry node){}

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

    public void visit(ASTRhoEntry node){}

}
