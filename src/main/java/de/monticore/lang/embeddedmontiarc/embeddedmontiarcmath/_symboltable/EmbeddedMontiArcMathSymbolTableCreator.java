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
// created by Michael von Wenckstern


package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathDelegatorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableHelper;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EmbeddedMontiArcMathSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EmbeddedMontiArcMathVisitor {

    // TODO doc
    private EmbeddedMontiArcMathDelegatorVisitor visitor = new EmbeddedMontiArcMathDelegatorVisitor();

    private EmbeddedMontiArcDynamicSymbolTableCreator emadSTC;
    private EmbeddedMontiArcBehaviorSymbolTableCreator emaBehaviorSTC;
    private EmbeddedMontiArcMathSymbolTableCreatorTOP emamSTC;
    private MathSymbolTableCreator mathSTC;

    public EmbeddedMontiArcMathSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        initSuperSTC(resolvingConfig);
    }

    public EmbeddedMontiArcMathSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC(resolvingConfig);
    }

    protected void initSuperSTC(final ResolvingConfiguration resolvingConfig) {
        // create symbol table creators
        this.emadSTC = new EmbeddedMontiArcDynamicSymbolTableCreator(resolvingConfig, scopeStack);
        this.emaBehaviorSTC = new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, scopeStack);
        this.mathSTC = new MathSymbolTableCreator(resolvingConfig, scopeStack);
        // assign to delegator visitor
        visitor.setEmbeddedMontiArcMathVisitor(emamSTC);
        visitor.setEmbeddedMontiArcVisitor(emadSTC);
        visitor.setEmbeddedMontiArcDynamicVisitor(emadSTC);
        visitor.setEmbeddedMontiArcBehaviorVisitor(emaBehaviorSTC);
        visitor.setMathVisitor(mathSTC);
        visitor.setExpressionsBasisVisitor(mathSTC);
        visitor.setCommonExpressionsVisitor(mathSTC);
        visitor.setAssignmentExpressionsVisitor(mathSTC);
        visitor.setMatrixExpressionsVisitor(mathSTC);
        visitor.setMatrixVisitor(mathSTC);
        visitor.setTypes2Visitor(mathSTC);
        visitor.setCommon2Visitor(emamSTC);
    }

    /**
     * Creates the symbol table starting from the <code>rootNode</code> and
     * returns the first scope that was created.
     *
     * @param rootNode the root node
     * @return the first scope that was created
     */
    public Scope createFromAST(ASTEMACompilationUnit rootNode) {
        Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EmbeddedMontiArcMathSymbolTableCreatorTOP symbol table: top ast node is null");
        rootNode.accept(visitor);
        return getFirstCreatedScope();
    }

    @Override
    public MutableScope getFirstCreatedScope() {
        return emadSTC.getFirstCreatedScope();
    }

    private EmbeddedMontiArcMathVisitor realThis = this;

    public EmbeddedMontiArcMathVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EmbeddedMontiArcMathVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
            visitor.setRealThis(realThis);
        }
    }

    protected EmbeddedMontiArcSymbolTableCreator getEmaSTC() {
        return emadSTC;
    }

    protected EmbeddedMontiArcDynamicSymbolTableCreator getEmadSTC() {
        return emadSTC;
    }

    protected EmbeddedMontiArcBehaviorSymbolTableCreator getEmaBehaviorSTC() {
        return emaBehaviorSTC;
    }

    protected EmbeddedMontiArcMathSymbolTableCreatorTOP getEmamSTC() {
        return emamSTC;
    }

    protected MathSymbolTableCreator getMathSTC() {
        return mathSTC;
    }
}
