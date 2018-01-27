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

import de.monticore.AmbiguityException;
import de.monticore.ast.ASTNode;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchCompilationUnit;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainCompilationUnit;
import de.monticore.lang.monticar.emadl._ast.ASTBehaviorEmbedding;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguageFamily;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.nio.file.Path;
import java.util.Collection;
import java.util.Optional;

public class LanguageFamilyModelLoader implements EMADLVisitor {

    private GlobalScope globalScope;
    private ResolvingConfiguration resolvingConfiguration;
    private ModelPath modelPath;

    private ASTEMADLCompilationUnit emadlAst;
    private ASTCNNArchCompilationUnit archAst;
    private ASTCNNTrainCompilationUnit trainAst;


    public LanguageFamilyModelLoader(Path directoryPath) {
        this.modelPath = new ModelPath(directoryPath.toAbsolutePath());
        this.resolvingConfiguration = new ResolvingConfiguration();
        this.globalScope = new GlobalScope(modelPath, EMADLLanguageFamily.INSTANCE);
    }

    public void loadEMADLModel(String qualifiedName){

        Collection<? extends ASTNode> models = EMADLLanguageFamily.EMADL_LANGUAGE.getModelLoader()
                .loadModelsIntoScope(qualifiedName, modelPath, globalScope, resolvingConfiguration);

        if (models.size() > 1) {
            throw new AmbiguityException("0xA4092 Multiple models were found with name '" + qualifiedName + "'");
        }
        else {
            Optional<? extends ASTNode> optModel = models.stream().findFirst();
            if (optModel.isPresent()){
                emadlAst = (ASTEMADLCompilationUnit)optModel.get();
                handle(emadlAst);
            }
            else {
                throw new RuntimeException("EMADL Model loading failed");
            }
        }
    }


    @Override
    public void visit(ASTBehaviorEmbedding node) {
        /*archAst = (ASTCNNArchCompilationUnit) node.getArchitectureEmbedding().getArchitectureSymbol().getAstNode().get();
        trainAst = (ASTCNNTrainCompilationUnit) node.getTrainingEmbedding().getTrainConfigSymbol().getAstNode().get();*/
    }



    public GlobalScope getGlobalScope() {
        return globalScope;
    }

    public ResolvingConfiguration getResolvingConfiguration() {
        return resolvingConfiguration;
    }

    public ModelPath getModelPath() {
        return modelPath;
    }

    public ASTEMADLCompilationUnit getEmadlAst() {
        return emadlAst;
    }

    public ASTCNNArchCompilationUnit getArchAst() {
        return archAst;
    }

    public ASTCNNTrainCompilationUnit getTrainAst() {
        return trainAst;
    }
}
