/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchCompilationUnit;
import de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchNode;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class ModifiedEMADynamicSymbolTableCreator extends EmbeddedMontiArcDynamicSymbolTableCreator {

    private ModifiedExpandedInstanceSymbolCreator instanceSymbolCreator = new ModifiedExpandedInstanceSymbolCreator();

    public ModifiedEMADynamicSymbolTableCreator(ResolvingConfiguration resolverConfig, MutableScope enclosingScope) {
        super(resolverConfig, enclosingScope);
    }

    public ModifiedEMADynamicSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    public ModifiedExpandedInstanceSymbolCreator getInstanceSymbolCreator() {
        return instanceSymbolCreator;
    }

    public void visit(ASTNode node){
        Log.info("TEST","TEST_MOD");
        Log.info(node.toString(),"TEST_MOD");
    }

    public void visit(ASTCNNArchNode node){
        Log.info("TEST","TEST_UNIT");
        Log.info(node.toString(),"TEST_UNIT");
    }
    @Override
    public void endVisit(ASTEMACompilationUnit node) {
        this.removeCurrentScope();
        if (!this.aboartVisitComponent) {
            Log.debug("endVisit of " + node.getComponent().getSymbolOpt().get().getFullName(), "SymbolTableCreator:");
            getInstanceSymbolCreator().createInstances((EMAComponentSymbol)(Log.errorIfNull(node.getComponent().getSymbolOpt().orElse(null))),node.getComponent().getSymbolOpt().get().getName());
        }
    }

}
