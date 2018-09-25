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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreatorTOP;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchSymbolTableCreator;
import de.monticore.lang.monticar.emadl._ast.ASTMathStatements;
import de.monticore.lang.monticar.emadl._visitor.EMADLDelegatorVisitor;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EMADLSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EMADLVisitor {
    
    private final EMADLDelegatorVisitor visitor = new EMADLDelegatorVisitor();

    private EmbeddedMontiArcSymbolTableCreator emaSTC;
    private EmbeddedMontiArcMathSymbolTableCreatorTOP emamSTC;
    private CNNArchSymbolTableCreator cnnArchSTC;


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
        this.emaSTC = new ModifiedEMASymbolTableCreator(resolvingConfig, scopeStack);//new ModifiedEMASymbolTableCreator(resolvingConfig, scopeStack);
        this.cnnArchSTC = new CNNArchSymbolTableCreator(resolvingConfig, scopeStack);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, scopeStack);

        visitor.setEmbeddedMontiArcVisitor(emaSTC);
        visitor.setEmbeddedMontiArcBehaviorVisitor(
                new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack));
        visitor.setEmbeddedMontiArcMathVisitor(emamSTC);

        visitor.setEMADLVisitor(this);
        visitor.setCNNArchVisitor(cnnArchSTC);

        visitor.setMathVisitor(cnnArchSTC.getMathSTC());
        visitor.setMatrixVisitor(cnnArchSTC.getMathSTC());
        visitor.setMatrixExpressionsVisitor(cnnArchSTC.getMathSTC());

        visitor.setExpressionsBasisVisitor(cnnArchSTC.getMathSTC());
        visitor.setCommonExpressionsVisitor(cnnArchSTC.getMathSTC());
        visitor.setTypes2Visitor(cnnArchSTC.getMathSTC());
        visitor.setAssignmentExpressionsVisitor(cnnArchSTC.getMathSTC());
    }

/**
     * Creates the symbol table starting from the <code>rootNode</code> and
     * returns the first scope that was created.
     *
     * @param rootNode the root node
     * @return the first scope that was created
     */


    public Scope createFromAST(ASTEMAMCompilationUnit rootNode) {
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

    public void endVisit(ASTMathStatements ast) {
        addToScopeAndLinkWithNode(new EMADLMathStatementsSymbol("MathStatements", ast), ast);
    }
}