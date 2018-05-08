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
        CNNTrainCompilationUnitSymbol compilationUnitSymbol = (CNNTrainCompilationUnitSymbol) ast.getSymbol().get();
        compilationUnitSymbol.setConfiguration((ConfigurationSymbol) ast.getConfiguration().getSymbol().get());
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
        for (ASTEntry nodeParam : node.getValue().getParams()) {
            OptimizerParamSymbol param = new OptimizerParamSymbol();
            OptimizerParamValueSymbol valueSymbol = (OptimizerParamValueSymbol) nodeParam.getValue().getSymbol().get();
            param.setValue(valueSymbol);
            configuration.getOptimizer().getOptimizerParamMap().put(nodeParam.getName(), param);;
        }

    }

    @Override
    public void endVisit(ASTNumEpochEntry node) {
        NumEpochSymbol symbol = new NumEpochSymbol();
        Integer value_as_int = node.getValue().getNumber().getUnitNumber().get().getNumber().get().getDividend().intValue();
        symbol.setValue(value_as_int);
        addToScopeAndLinkWithNode(symbol, node);
        configuration.setNumEpoch(symbol);
    }

    @Override
    public void endVisit(ASTBatchSizeEntry node) {
        BatchSizeSymbol symbol = new BatchSizeSymbol();
        Integer value_as_int = node.getValue().getNumber().getUnitNumber().get().getNumber().get().getDividend().intValue();
        symbol.setValue(value_as_int);
        addToScopeAndLinkWithNode(symbol, node);
        configuration.setBatchSize(symbol);
    }

    @Override
    public void endVisit(ASTLoadCheckpointEntry node) {
        LoadCheckpointSymbol symbol = new LoadCheckpointSymbol();
        if (node.getValue().getTRUE().isPresent()){
            symbol.setValue(true);
        }
        else if (node.getValue().getFALSE().isPresent()){
           symbol.setValue(false);
        }
         configuration.setLoadCheckpoint(symbol);
    }

    @Override
    public void endVisit(ASTNormalizeEntry node) {
        NormalizeSymbol symbol = new NormalizeSymbol();
        if (node.getValue().getTRUE().isPresent()){
            symbol.setValue(true);
        }
        else if (node.getValue().getFALSE().isPresent()){
            symbol.setValue(false);
        }
        configuration.setNormalize(symbol);
    }


    @Override
    public void endVisit(ASTLRPolicyValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
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
    public void endVisit(ASTNumberValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        Double number = node.getNumber().getUnitNumber().get().getNumber().get().doubleValue();
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTIntegerValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        Integer number = node.getNumber().getUnitNumber().get().getNumber().get().getDividend().intValue();
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTBooleanValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        if (node.getTRUE().isPresent()){
            value.setValue(true);
        }
        else if (node.getFALSE().isPresent()){
            value.setValue(false);
        }
        addToScopeAndLinkWithNode(value, node);
    }

}
