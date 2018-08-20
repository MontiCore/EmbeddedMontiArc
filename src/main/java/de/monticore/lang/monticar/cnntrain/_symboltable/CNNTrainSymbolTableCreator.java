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
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class CNNTrainSymbolTableCreator extends CNNTrainSymbolTableCreatorTOP {

    private String compilationUnitPackage = "";
    private ConfigurationSymbol configuration;


    public CNNTrainSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                      final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public CNNTrainSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                      final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }


    @Override
    public void visit(final ASTCNNTrainCompilationUnit compilationUnit) {
        Log.debug("Building Symboltable for Script: " + compilationUnit.getName(),
                CNNTrainSymbolTableCreator.class.getSimpleName());

        List<ImportStatement> imports = new ArrayList<>();

        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);

        putOnStack(artifactScope);

        CNNTrainCompilationUnitSymbol compilationUnitSymbol = new CNNTrainCompilationUnitSymbol(compilationUnit.getName());
        addToScopeAndLinkWithNode(compilationUnitSymbol, compilationUnit);
    }

    @Override
    public void endVisit(ASTCNNTrainCompilationUnit ast) {
        CNNTrainCompilationUnitSymbol compilationUnitSymbol = (CNNTrainCompilationUnitSymbol) ast.getSymbolOpt().get();
        compilationUnitSymbol.setConfiguration((ConfigurationSymbol) ast.getConfiguration().getSymbolOpt().get());
        setEnclosingScopeOfNodes(ast);
    }


    @Override
    public void visit(final ASTConfiguration node){
        configuration = new ConfigurationSymbol();
        addToScopeAndLinkWithNode(configuration , node);
    }

    @Override
    public void endVisit(final ASTConfiguration trainingConfiguration) {
        removeCurrentScope();
    }

    @Override
    public void visit(ASTOptimizerEntry node) {
        OptimizerSymbol optimizer = new OptimizerSymbol(node.getValue().getName());
        configuration.setOptimizer(optimizer);
        addToScopeAndLinkWithNode(optimizer, node);
    }

    @Override
    public void endVisit(ASTOptimizerEntry node) {
        for (ASTEntry nodeParam : node.getValue().getParamsList()) {
            OptimizerParamSymbol param = new OptimizerParamSymbol();
            OptimizerParamValueSymbol valueSymbol = (OptimizerParamValueSymbol) nodeParam.getValue().getSymbolOpt().get();
            param.setValue(valueSymbol);
            configuration.getOptimizer().getOptimizerParamMap().put(nodeParam.getName(), param);;
        }

    }

    @Override
    public void endVisit(ASTNumEpochEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        Integer value_as_int = getIntegerFromNumber(node.getValue());
        value.setValue(value_as_int);
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTBatchSizeEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        Integer value_as_int = getIntegerFromNumber(node.getValue());
        value.setValue(value_as_int);
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTLoadCheckpointEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        value.setValue(getBooleanForValue(node.getValue()));
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTNormalizeEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        value.setValue(getBooleanForValue(node.getValue()));
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTTrainContextEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        if (node.getValue().isPresentCpu()){
            value.setValue(Context.CPU);
        }
        else {
            value.setValue(Context.GPU);
        }
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTEvalMetricEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        if (node.getValue().isPresentAccuracy()){
            value.setValue(EvalMetric.ACCURACY);
        }
        else if (node.getValue().isPresentCrossEntropy()){
            value.setValue(EvalMetric.CROSS_ENTROPY);
        }
        else if (node.getValue().isPresentF1()){
            value.setValue(EvalMetric.F1);
        }
        else if (node.getValue().isPresentMae()){
            value.setValue(EvalMetric.MAE);
        }
        else if (node.getValue().isPresentMse()){
            value.setValue(EvalMetric.MSE);
        }
        else if (node.getValue().isPresentRmse()){
            value.setValue(EvalMetric.RMSE);
        }
        else if (node.getValue().isPresentTopKAccuracy()){
            value.setValue(EvalMetric.TOP_K_ACCURACY);
        }
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTLRPolicyValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        if (node.isPresentFixed()){
            value.setValue(LRPolicy.FIXED);
        }
        else if (node.isPresentExp()){
            value.setValue(LRPolicy.EXP);
        }
        else if (node.isPresentInv()){
            value.setValue(LRPolicy.INV);
        }
        else if (node.isPresentStep()){
            value.setValue(LRPolicy.STEP);
        }
        else if (node.isPresentSigmoid()){
            value.setValue(LRPolicy.SIGMOID);
        }
        else if (node.isPresentPoly()){
            value.setValue(LRPolicy.POLY);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTNumberValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        Double number = node.getNumberWithUnit().getNumber().get();
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTIntegerValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        Integer number = getIntegerFromNumber(node);
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTBooleanValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        if (node.isPresentTRUE()){
            value.setValue(true);
        }
        else if (node.isPresentFALSE()){
            value.setValue(false);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    private int getIntegerFromNumber(ASTIntegerValue value) {
        return value.getNumberWithUnit().getNumber().get().intValue();
    }

    private boolean getBooleanForValue(ASTBooleanValue value2) {
        if (value2.isPresentTRUE()) {
            return true;
        }
        return false;
    }

}
