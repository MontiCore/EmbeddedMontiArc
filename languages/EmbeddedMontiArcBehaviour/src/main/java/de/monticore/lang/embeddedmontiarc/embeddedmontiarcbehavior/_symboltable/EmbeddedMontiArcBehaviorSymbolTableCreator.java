/* (c) https://github.com/MontiCore/monticore */
/* generated by template symboltable.SymbolTableCreator*/




package de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._visitor.EmbeddedMontiArcBehaviorDelegatorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._visitor.EmbeddedMontiArcBehaviorVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EmbeddedMontiArcBehaviorSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EmbeddedMontiArcBehaviorVisitor {

    public EmbeddedMontiArcBehaviorSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        initSuperSTC();
    }

    public EmbeddedMontiArcBehaviorSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC();
    }

    private void initSuperSTC() {
        // TODO doc
        // visitor.set_de_monticore_lang_embeddedmontiarc_embeddedmontiarcmath__visitor_EmbeddedMontiArcMathVisitor(de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathSymbolTableCreator(resolvingConfig, scopeStack));
        // visitor.set_de_monticore_lang_embeddedmontiarc_embeddedmontiarcbehavior__visitor_EmbeddedMontiArcBehaviorVisitor(de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._visitor.EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack));
        // visitor.set_de_monticore_lang_montiarc_math__visitor_MathVisitor(de.monticore.lang.montiarc.math._visitor.MathSymbolTableCreator(resolvingConfig, scopeStack));
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
        rootNode.accept(realThis);
        return getFirstCreatedScope();
    }

    private EmbeddedMontiArcBehaviorVisitor realThis = this;

    public EmbeddedMontiArcBehaviorVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EmbeddedMontiArcBehaviorVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
        }
    }

}