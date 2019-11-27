/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._visitor.EmbeddedMontiArcBehaviorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreatorTOP;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure.ModifiedEMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchSymbolTableCreator;
import de.monticore.lang.monticar.emadl._ast.ASTBehaviorEmbedding;
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

    private EmbeddedMontiArcMathSymbolTableCreatorTOP emamSTC;
    private CNNArchSymbolTableCreator cnnArchSTC;
    private MathOptSymbolTableCreator mathOptSTC;
    private EmbeddedMontiArcDynamicSymbolTableCreator emadSTC;
    private EmbeddedMontiArcBehaviorVisitor emaBehaviorSTC;

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
        this.cnnArchSTC = new CNNArchSymbolTableCreator(resolvingConfig, scopeStack);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, scopeStack);
        this.mathOptSTC = new MathOptSymbolTableCreator(resolvingConfig, scopeStack);
        this.emadSTC = new ModifiedEMADynamicSymbolTableCreator(resolvingConfig, scopeStack);
        this.emadSTC.setInstanceSymbolCreator(new ModifiedEMAComponentInstanceSymbolCreator()); //Use an instance symbo, creator that adds math statement to instances
        this.emaBehaviorSTC = new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack);

        visitor.setEMADLVisitor(this);
        visitor.setCNNArchVisitor(cnnArchSTC);

        visitor.setEmbeddedMontiArcMathVisitor(emamSTC);
        visitor.setEmbeddedMontiArcVisitor(emadSTC);
        visitor.setEmbeddedMontiArcDynamicVisitor(emadSTC);
        visitor.setEmbeddedMontiArcBehaviorVisitor(emaBehaviorSTC);
        visitor.setMathVisitor(mathOptSTC);
        visitor.setExpressionsBasisVisitor(mathOptSTC);
        visitor.setCommonExpressionsVisitor(mathOptSTC);
        visitor.setAssignmentExpressionsVisitor(mathOptSTC);
        visitor.setMatrixExpressionsVisitor(mathOptSTC);
        visitor.setMatrixVisitor(mathOptSTC);
        visitor.setTypes2Visitor(mathOptSTC);
        visitor.setMathOptVisitor(mathOptSTC);

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
        Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EMADLSymbolTableCreator symbol table: top ast node is null");
        rootNode.accept(visitor);
        return getFirstCreatedScope();
    }

    @Override
    public MutableScope getFirstCreatedScope() {
        return emadSTC.getFirstCreatedScope();
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

    public void endVisit(ASTBehaviorEmbedding ast) {
        if(ast.isPresentArchitecture()){
            //processed in handle/visit/endVisit of ASTArchitecture and ASTArchBody
        }else if(ast.getStatementList().size() > 0) {
            addToScopeAndLinkWithNode(new EMADLMathStatementsSymbol("MathStatements", ast.getStatementList()), ast);
        }
    }


}
