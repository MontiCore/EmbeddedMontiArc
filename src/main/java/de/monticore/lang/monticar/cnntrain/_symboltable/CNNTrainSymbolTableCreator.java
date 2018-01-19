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

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

public class CNNTrainSymbolTableCreator extends CNNTrainSymbolTableCreatorTOP {

    private String compilationUnitPackage = "";
    private TrainingConfigurationSymbol configuration;


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
        Log.debug("Building Symboltable for Script: " + compilationUnit.getTrainingConfiguration().getName(),
                CNNTrainSymbolTableCreator.class.getSimpleName());

        List<ImportStatement> imports = new ArrayList<>();

        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);

        putOnStack(artifactScope);
    }

    @Override
    public void visit(final ASTTrainingConfiguration node){
        configuration = new TrainingConfigurationSymbol(node.getName());
        addToScopeAndLinkWithNode(configuration , node);
    }

    @Override
    public void endVisit(final ASTTrainingConfiguration trainingConfiguration) {
        removeCurrentScope();
    }

    @Override
    public void endVisit(ASTEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue((ValueSymbol) node.getValue().getSymbol().get());
        addToScopeAndLinkWithNode(entry, node);

        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTDataValue node){
        ValueSymbol value;
        if (node.getDataVariable().isPresent()){
            value = (ValueSymbol) node.getDataVariable().get().getSymbol().get();
        }
        else {
            value = (ValueSymbol) node.getPathValue().get().getSymbol().get();
        }
        node.setSymbol(value);
    }

    @Override
    public void endVisit(ASTPathValue node) {
        ValueSymbol value = new ValueSymbol();
        value.setValue(node.getPath().getValue().replaceAll("\"", ""));
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTNumberValue node) {
        ValueSymbol value = new ValueSymbol();
        Double number = node.getNumber().getUnitNumber().get().getNumber().get().doubleValue();
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTIntegerValue node) {
        ValueSymbol value = new ValueSymbol();
        Integer number = node.getNumber().getUnitNumber().get().getNumber().get().getDividend().intValue();
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTBooleanValue node) {
        ValueSymbol value = new ValueSymbol();
        if (node.getTRUE().isPresent()){
            value.setValue(true);
        }
        else if (node.getFALSE().isPresent()){
            value.setValue(false);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTContextValue node) {
        ValueSymbol value = new ValueSymbol();
        if (node.getCPU().isPresent()){
            value.setValue(Context.CPU);
        }
        else if (node.getGPU().isPresent()){
            value.setValue(Context.GPU);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTEvalMetricValue node) {
        ValueSymbol value = new ValueSymbol();
        if (node.getAccuracy().isPresent()){
            value.setValue(EvalMetric.ACCURACY);
        }
        else if (node.getCrossEntropy().isPresent()){
            value.setValue(EvalMetric.CROSS_ENTROPY);
        }
        else if (node.getF1().isPresent()){
            value.setValue(EvalMetric.F1);
        }
        else if (node.getMae().isPresent()){
            value.setValue(EvalMetric.MAE);
        }
        else if (node.getMse().isPresent()){
            value.setValue(EvalMetric.MSE);
        }
        else if (node.getRmse().isPresent()){
            value.setValue(EvalMetric.RMSE);
        }
        else if (node.getTopKAccuracy().isPresent()){
            value.setValue(EvalMetric.TOP_K_ACCURACY);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTLRPolicyValue node) {
        ValueSymbol value = new ValueSymbol();
        if (node.getFixed().isPresent()){
            value.setValue(LRPolicy.FIXED);
        }
        else if (node.getExp().isPresent()){
            value.setValue(LRPolicy.EXP);
        }
        else if (node.getInv().isPresent()){
            value.setValue(LRPolicy.INV);
        }
        else if (node.getStep().isPresent()){
            value.setValue(LRPolicy.STEP);
        }
        else if (node.getSigmoid().isPresent()){
            value.setValue(LRPolicy.SIGMOID);
        }
        else if (node.getPoly().isPresent()){
            value.setValue(LRPolicy.POLY);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTLoadingModeValue node) {
        ValueSymbol value = new ValueSymbol();
        if (node.getLoadAndTrain().isPresent()){
            value.setValue(LoadingMode.LOAD_AND_TRAIN);
        }
        else if (node.getLoadOnly().isPresent()){
            value.setValue(LoadingMode.LOAD_ONLY);
        }
        else if (node.getNoLoad().isPresent()){
            value.setValue(LoadingMode.NO_LOAD);
        }
        else if (node.getOverwrite().isPresent()){
            value.setValue(LoadingMode.OVERWRITE);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    public void endVisit(ASTOptimizerValue node){
        ValueSymbol value = new ValueSymbol();
        value.setValue(node.getName());
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTDataVariable node){
        NameValueSymbol variableSymbol = new NameValueSymbol(node.getName());
        addToScopeAndLinkWithNode(variableSymbol, node);
    }
}
