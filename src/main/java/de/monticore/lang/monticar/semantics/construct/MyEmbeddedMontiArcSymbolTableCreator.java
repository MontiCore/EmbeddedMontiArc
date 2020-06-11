/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreatorTOP;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure.ModifiedEMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathDelegatorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;
import java.util.List;
import java.util.Set;

public class MyEmbeddedMontiArcSymbolTableCreator extends EmbeddedMontiArcMathSymbolTableCreator {

    private EmbeddedMontiArcMathDelegatorVisitor visitor = new EmbeddedMontiArcMathDelegatorVisitor();
    private MyEmbeddedMontiArcDynamicSymbolTableCreator emadSTC;
    private EmbeddedMontiArcBehaviorSymbolTableCreator emaBehaviorSTC;
    private EmbeddedMontiArcMathSymbolTableCreatorTOP emamSTC;
    private MathOptSymbolTableCreator mathOptSTC;
    private EmbeddedMontiArcMathVisitor realThis = this;
    private Set<ComponentReplacement> componentReplacements;

    public MyEmbeddedMontiArcSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC(resolvingConfig);
    }

    public MyEmbeddedMontiArcSymbolTableCreator(ResolvingConfiguration resolverConfig, MutableScope enclosingScope) {
        super(resolverConfig, enclosingScope);
        initSuperSTC(resolverConfig);
    }

    @Override
    protected void initSuperSTC(ResolvingConfiguration resolvingConfig) {
        visitor = new EmbeddedMontiArcMathDelegatorVisitor();
        this.emadSTC = new MyEmbeddedMontiArcDynamicSymbolTableCreator(resolvingConfig, this.scopeStack);
        this.emadSTC.setComponentReplacements(componentReplacements);
        this.emadSTC.setInstanceSymbolCreator(new ModifiedEMAComponentInstanceSymbolCreator());
        this.emaBehaviorSTC = new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, this.scopeStack);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, this.scopeStack);
        this.mathOptSTC = new MathOptSymbolTableCreator(resolvingConfig, this.scopeStack);
        this.visitor.setEmbeddedMontiArcMathVisitor(this.emamSTC);
        this.visitor.setEmbeddedMontiArcVisitor(this.emadSTC);
        this.visitor.setEmbeddedMontiArcDynamicVisitor(this.emadSTC);
        this.visitor.setEmbeddedMontiArcBehaviorVisitor(this.emaBehaviorSTC);
        this.visitor.setMathVisitor(this.mathOptSTC);
        this.visitor.setExpressionsBasisVisitor(this.mathOptSTC);
        this.visitor.setCommonExpressionsVisitor(this.mathOptSTC);
        this.visitor.setAssignmentExpressionsVisitor(this.mathOptSTC);
        this.visitor.setMatrixExpressionsVisitor(this.mathOptSTC);
        this.visitor.setMatrixVisitor(this.mathOptSTC);
        this.visitor.setTypes2Visitor(this.mathOptSTC);
        this.visitor.setMathOptVisitor(this.mathOptSTC);
        this.visitor.setCommon2Visitor(this.emamSTC);
    }

    public Scope createFromAST(ASTEMACompilationUnit rootNode) {
        Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EmbeddedMontiArcMathSymbolTableCreatorTOP symbol table: top ast node is null");
        rootNode.accept(this.visitor);
        return this.getFirstCreatedScope();
    }

    @Override
    public MutableScope getFirstCreatedScope() {
        return this.emadSTC.getFirstCreatedScope();
    }

    @Override
    public EmbeddedMontiArcMathVisitor getRealThis() {
        return this.realThis;
    }

    @Override
    public void setRealThis(EmbeddedMontiArcMathVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
            this.visitor.setRealThis(realThis);
        }

    }

    @Override
    protected EmbeddedMontiArcSymbolTableCreator getEmaSTC() {
        return this.emadSTC;
    }

    @Override
    protected EmbeddedMontiArcDynamicSymbolTableCreator getEmadSTC() {
        return this.emadSTC;
    }

    public void setComponentReplacements(Set<ComponentReplacement> componentReplacements) {
        this.componentReplacements = componentReplacements;
        this.emadSTC.setComponentReplacements(componentReplacements);
    }
}
