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
package de.monticore.lang.monticar.emadl.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._visitor.EmbeddedMontiArcInheritanceVisitor;
import de.monticore.lang.monticar.cnnarch._ast.*;
import de.monticore.lang.monticar.cnnarch._visitor.CNNArchInheritanceVisitor;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainCompilationUnit;
import de.monticore.lang.monticar.cnntrain._ast.ASTTrainingConfiguration;
import de.monticore.lang.monticar.cnntrain._visitor.CNNTrainInheritanceVisitor;
import de.monticore.lang.monticar.emadl._ast.*;
import de.monticore.lang.monticar.emadl._visitor.EMADLInheritanceVisitor;
import de.monticore.symboltable.GlobalScope;

import java.util.LinkedList;
import java.util.List;

public class TemplateData implements EMADLInheritanceVisitor,
                                     CNNArchInheritanceVisitor,
                                     CNNTrainInheritanceVisitor,
                                     EmbeddedMontiArcInheritanceVisitor {

    private GlobalScope globalScope;

    private ASTTrainingConfiguration config;
    private List<ASTLayer> layers;
    private List<ASTMainLayer> mainLayers;
    private ASTContext context;
    private ASTTarget target;
    private ASTOutputLayer outputLayer;
    private PortArraySymbol inputPort;
    private PortArraySymbol outputPort;



    public TemplateData(GlobalScope globalScope) {
        this.globalScope = globalScope;
    }

    @Override
    public void handle(ASTArchitectureEmbedding node) {
        handle((ASTCNNArchCompilationUnit) node.getArchitectureSymbol().getAstNode().get());
        inputPort = node.getInputSymbol();
        outputPort = node.getOutputSymbol();
    }

    @Override
    public void handle(ASTTrainingEmbedding node) {
        handle((ASTCNNTrainCompilationUnit) node.getTrainConfigSymbol().getAstNode().get());
    }

    @Override
    public void visit(ASTArchitecture node) {
        mainLayers = node.getMainLayers();

        layers = new LinkedList<ASTLayer>();
        layers.addAll(mainLayers);
        layers.add(node.getOutputLayer());
    }

    @Override
    public void visit(ASTOutputLayer node) {
        outputLayer = node;
    }

    @Override
    public void visit(ASTOption node) {
        String name = node.getName();
        switch (name){
            case "target":
                target = (ASTTarget) node.getValue();
                break;
            case "context":
                context = (ASTContext) node.getValue();
                break;
        }
    }

    @Override
    public void visit(ASTTrainingConfiguration node) {
        config = node;
    }

    public ASTTrainingConfiguration getConfig() {
        return config;
    }

    //returns mainLayers + outputLayer
    public List<ASTLayer> getLayers() {
        return layers;
    }


    public ASTOutputLayer getOutputLayer() {
        return outputLayer;
    }

    public ASTContext getContext() {
        if (context == null) {
            context = ASTContext.CPU;
        }
        return context;
    }

    public ASTTarget getTarget() {
        if (target == null) {
            target = ASTTarget.PYTHON;
        }
        return target;
    }

    public List<ASTMainLayer> getMainLayers() {
        return mainLayers;
    }

    public PortArraySymbol getInputPort() {
        return inputPort;
    }

    public PortArraySymbol getOutputPort() {
        return outputPort;
    }

    public GlobalScope getGlobalScope() {
        return globalScope;
    }
}
