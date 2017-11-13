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
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EMAExpandedComponentInstanceKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchSymbolTableCreator;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainSymbolTableCreator;
import de.monticore.lang.monticar.emadl._ast.ASTArchitectureEmbedding;
import de.monticore.lang.monticar.emadl._ast.ASTBehaviorEmbedding;
import de.monticore.lang.monticar.emadl._ast.ASTTrainingEmbedding;
import de.monticore.lang.monticar.emadl._visitor.CommonEMADLDelegatorVisitor;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EMADLSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EMADLVisitor {
    
    private final CommonEMADLDelegatorVisitor visitor = new CommonEMADLDelegatorVisitor();

    private EmbeddedMontiArcSymbolTableCreator emaSTC;
    private Scope componentScope;
    

    public EMADLSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        initSuperSTC(resolvingConfig);
    }

    public EMADLSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC(resolvingConfig);
    }

    private void initSuperSTC(final ResolvingConfiguration resolvingConfig) {
        this.emaSTC = new EmbeddedMontiArcSymbolTableCreator(resolvingConfig, scopeStack);

        visitor.set_de_monticore_lang_embeddedmontiarc_embeddedmontiarc__visitor_EmbeddedMontiArcVisitor(emaSTC);
        visitor.set_de_monticore_lang_monticar_emadl__visitor_EMADLVisitor(this);
        visitor.set_de_monticore_lang_embeddedmontiarc_embeddedmontiarcbehavior__visitor_EmbeddedMontiArcBehaviorVisitor(
                new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack));
        visitor.set_de_monticore_lang_monticar_cnnarch__visitor_CNNArchVisitor(new CNNArchSymbolTableCreator(resolvingConfig, scopeStack));
        visitor.set_de_monticore_lang_monticar_cnntrain__visitor_CNNTrainVisitor(new CNNTrainSymbolTableCreator(resolvingConfig, scopeStack));
    }

/**
     * Creates the symbol table starting from the <code>rootNode</code> and
     * returns the first scope that was created.
     *
     * @param rootNode the root node
     * @return the first scope that was created
     */


    public Scope createFromAST(de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit rootNode) {
        Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EMADLSymbolTableCreator symbol table: top ast node is null");
        rootNode.accept(visitor);
        return getFirstCreatedScope();
    }

    @Override
    public MutableScope getFirstCreatedScope() {
        return emaSTC.getFirstCreatedScope();
    }

    private EMADLVisitor realThis = this;

    public EMADLVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EMADLVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
            visitor.setRealThis(realThis);
        }
    }


    @Override
    public void visit(de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit ast) {
        EMADLCompilationUnitSymbol emadlCompilationUnit = create_EMADLCompilationUnit(ast);
        initialize_EMADLCompilationUnit(emadlCompilationUnit, ast);
        addToScopeAndLinkWithNode(emadlCompilationUnit, ast);
    }

    protected EMADLCompilationUnitSymbol create_EMADLCompilationUnit(de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit ast) {
        return new EMADLCompilationUnitSymbol("");
    }

    protected void initialize_EMADLCompilationUnit(EMADLCompilationUnitSymbol eMADLCompilationUnit, de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit ast) {

    }

    @Override
    public void visit(ASTBehaviorEmbedding node) {
        componentScope = emaSTC.currentScope().get();
        node.setEnclosingScope(componentScope);
    }

    @Override
    public void endVisit(ASTArchitectureEmbedding node) {
        node.setEnclosingScope(componentScope);
    }

    @Override
    public void visit(ASTTrainingEmbedding node) {
        node.setEnclosingScope(componentScope);
    }

}

