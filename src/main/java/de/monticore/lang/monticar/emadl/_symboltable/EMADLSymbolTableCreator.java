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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcArtifactScope;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.monticar.emadl._ast.*;
import de.monticore.lang.monticar.emadl._visitor.CommonEMADLDelegatorVisitor;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

public class EMADLSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EMADLVisitor {
    
    private final CommonEMADLDelegatorVisitor visitor = new CommonEMADLDelegatorVisitor();

    private EmbeddedMontiArcSymbolTableCreator emaSTC;
    private ArchitectureConstructorSymbol archConstructor;
    private ConfigConstructorSymbol configConstructor;
    String componentName;


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
        visitor.set_de_monticore_lang_embeddedmontiarc_embeddedmontiarcbehavior__visitor_EmbeddedMontiArcBehaviorVisitor(
                new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack));
        visitor.set_de_monticore_lang_monticar_emadl__visitor_EMADLVisitor(this);
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

    private EmbeddedMontiArcArtifactScope getArtifactScope(final Scope scope){
        Scope s = scope;
        while (!(s instanceof EmbeddedMontiArcArtifactScope)){
            s = s.getEnclosingScope().get();
        }
        return (EmbeddedMontiArcArtifactScope) s;
    }

    @Override
    public void visit(ASTEMADLCompilationUnit node) {
        componentName = node.getEMACompilationUnit().getComponent().getName();
    }

    @Override
    public void visit(ASTBehaviorEmbedding ast) {
        EMADLBehaviorSymbol sym = new EMADLBehaviorSymbol(componentName);

        addToScopeAndLinkWithNode(sym, ast);
    }

    @Override
    public void endVisit(ASTBehaviorEmbedding ast) {
        EMADLBehaviorSymbol sym = (EMADLBehaviorSymbol) ast.getSymbol().get();
        sym.setArchitectureConstructor(archConstructor);
        sym.setConfigConstructor(configConstructor);

        removeCurrentScope();

        //add reference to artifact scope
        EMADLBehaviorSymbolReference reference = new EMADLBehaviorSymbolReference(componentName, currentScope().get());
        getArtifactScope(currentScope().get())
                .add(reference);
    }

    @Override
    public void visit(ASTArchitectureConstructor node) {
        archConstructor = new ArchitectureConstructorSymbol(node.getName());
        addToScopeAndLinkWithNode(archConstructor, node);
    }

    @Override
    public void endVisit(ASTArchitectureConstructor node) {
        archConstructor.setArguments(node.getArguments());

        List<ArchPortSymbol> inputs = new ArrayList<>();
        for (ASTArchIOPort astPort : node.getInput().getPorts()){
            inputs.add((ArchPortSymbol) astPort.getSymbol().get());
        }
        archConstructor.setInputs(inputs);

        List<ArchPortSymbol> outputs = new ArrayList<>();
        for (ASTArchIOPort astPort : node.getOutput().getPorts()){
            outputs.add((ArchPortSymbol) astPort.getSymbol().get());
        }
        archConstructor.setOutputs(outputs);

        removeCurrentScope();
    }

    @Override
    public void visit(ASTConfigConstructor node) {
        configConstructor = new ConfigConstructorSymbol(node.getName());
        addToScopeAndLinkWithNode(configConstructor, node);
    }

    @Override
    public void endVisit(ASTConfigConstructor node) {
        configConstructor.setArguments(node.getArguments());

        removeCurrentScope();
    }

    public void endVisit(ASTArchIOPort node) {
        ArchPortSymbol sym = new ArchPortSymbol(node.getName());
        if (node.getAlias().isPresent()){
            sym.setAlias(node.getAlias().get());
        }
        addToScopeAndLinkWithNode(sym, node);
    }
}

