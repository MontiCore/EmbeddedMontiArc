/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
// created by Michael von Wenckstern


package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure.ModifiedEMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathDelegatorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableHelper;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EmbeddedMontiArcMathSymbolTableCreator extends EmbeddedMontiArcMathSymbolTableCreatorTOP {

    // TODO doc
    private EmbeddedMontiArcMathDelegatorVisitor visitor = new EmbeddedMontiArcMathDelegatorVisitor();

    private EmbeddedMontiArcDynamicSymbolTableCreator emadSTC;
    private EmbeddedMontiArcBehaviorSymbolTableCreator emaBehaviorSTC;
    private EmbeddedMontiArcMathSymbolTableCreatorTOP emamSTC;
    private MathOptSymbolTableCreator mathOptSTC;

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
        this.emadSTC.setInstanceSymbolCreator(new ModifiedEMAComponentInstanceSymbolCreator()); //Use an instance symbo, creator that adds math statement to instances

        this.emaBehaviorSTC = new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, scopeStack);
        this.mathOptSTC = new MathOptSymbolTableCreator(resolvingConfig, scopeStack);

        // assign to delegator visitor
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
        return  mathOptSTC;
    }

    protected MathSymbolTableCreator getMathOptSTC() {
        return  mathOptSTC;
    }

}
