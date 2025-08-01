/* (c) https://github.com/MontiCore/monticore */


package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._symboltable;

import de.monticore.lang.application.application._symboltable.ApplicationSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._ast.ASTEMAAplCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._visitor.EmbeddedMontiArcApplicationDelegatorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._visitor.EmbeddedMontiArcApplicationVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EmbeddedMontiArcApplicationSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EmbeddedMontiArcApplicationVisitor {

    // TODO doc
    private final EmbeddedMontiArcApplicationDelegatorVisitor visitor = new EmbeddedMontiArcApplicationDelegatorVisitor();

    private EmbeddedMontiArcSymbolTableCreator emaSTC;

    public EmbeddedMontiArcApplicationSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        initSuperSTC(resolvingConfig);
    }

    public EmbeddedMontiArcApplicationSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC(resolvingConfig);
    }

    private void initSuperSTC(final ResolvingConfiguration resolvingConfig) {
        this.emaSTC = new EmbeddedMontiArcSymbolTableCreator(resolvingConfig, scopeStack);

        visitor.setEmbeddedMontiArcApplicationVisitor(new EmbeddedMontiArcApplicationSymbolTableCreatorTOP(resolvingConfig, scopeStack));
        visitor.setEmbeddedMontiArcVisitor(emaSTC);
        visitor.setEmbeddedMontiArcBehaviorVisitor(
                new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack));
        visitor.setApplicationVisitor(new ApplicationSymbolTableCreator(resolvingConfig, scopeStack));
        //visitor.set_de_monticore_java_javadsl__visitor_JavaDSLVisitor(new JavaSymbolTableCreator(resolvingConfig, scopeStack));
    }

    /**
     * Creates the symbol table starting from the <code>rootNode</code> and
     * returns the first scope that was created.
     *
     * @param rootNode the root node
     * @return the first scope that was created
     */
    public Scope createFromAST(ASTEMAAplCompilationUnit rootNode) {
        Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EmbeddedMontiArcApplicationSymbolTableCreatorTOP symbol table: top ast node is null");
        rootNode.accept(visitor);
        return getFirstCreatedScope();
    }

    @Override
    public MutableScope getFirstCreatedScope() {
        return emaSTC.getFirstCreatedScope();
    }

    private EmbeddedMontiArcApplicationVisitor realThis = this;

    public EmbeddedMontiArcApplicationVisitor getRealThis() {
        return realThis;
    }

    /*@Override
    public void handle(de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._ast.ASTComponent node) {
        emaSTC.visit((de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent) node);
        emaSTC.traverse((de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent)node);
        emaSTC.endVisit((de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent)node);
    }*/

    @Override
    public void setRealThis(EmbeddedMontiArcApplicationVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
            visitor.setRealThis(realThis);
        }
    }

}
