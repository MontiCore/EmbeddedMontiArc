/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.lang.monticar.emadl._visitor.ModularNetworkVisitor;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class ModularCNNSymbolTableCreator extends CommonSymbolTableCreator implements ModularNetworkVisitor {


    private ModularNetworkVisitor realThis = this;

    public ModularCNNSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        this.initSuperSTC();
        Log.info("INIT","MCNNSTC_INIT_ENCLOSING-SCOPE");
    }

    public ModularCNNSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        this.initSuperSTC();
        Log.info("INIT","MCNNSTC_INIT_SCOPE-STACK");
    }

    public void initSuperSTC(){

    }

    public Scope createFromAST(ASTEMADLNode rootNode) {
        Log.errorIfNull(rootNode, "ModularCNNSymbolTableCreator symbol table: top ast node is null");
        rootNode.accept(this.realThis);
        return this.getFirstCreatedScope();
    }



    public Scope createFromAST(ASTEMACompilationUnit rootNode) {
        Log.errorIfNull(rootNode, "ModularCNNSymbolTableCreator symbol table: top ast node is null");
        rootNode.accept(this.realThis);
        return this.getFirstCreatedScope();
    }

    public ModularNetworkVisitor getRealThis() {
        return this.realThis;
    }

    public void setRealThis (ModularNetworkVisitor rt) {
        if (this.realThis != rt) {
            this.realThis =  rt;
        }
    }

    public void setRealThis (EMADLVisitor v) {
        if (v instanceof ModularNetworkVisitor){
            Log.info("MCNNSTC Set Real this EMADLVisitor pass","MCNNSTC_SET_REAL_THIS_INSTANCE");
            this.setRealThis((ModularNetworkVisitor) v);
        } else{
            Log.info("MCNNSTC Set Real this EMADLVisitor fail","MCNNSTC_SET_REAL_THIS_INSTANCE");
        }

    }



    @Override
    public void traverse(ASTNode node){
        Log.info("MCNNSTC","TRAVERSE_MCNNSTC");
    }

    @Override
    public void handle(ASTNode node) {
        Log.info("MCNNSTC","HANDLE_MCNNSTC");
    }

    @Override
    public void visit(ASTNode node) {
        Log.info("MCNNSTC","VISIT_MCNNSTC");
        Log.info(node.toString(),"VISIT_MCNNSTC");

        NetworkStructureScanner nss = new NetworkStructureScanner();
        nss.scanStructure(node);
    }

    @Override
    public void endVisit(ASTNode node){
        Log.info("MCNNSTC","END_MCNNSTC");
    }


    @Override
    public void visit(ASTEMACompilationUnit node){
        Log.info("MCNNSTC","VISIT_MCNNSTC_COMP");



        NetworkStructureScanner nss = new NetworkStructureScanner();
        nss.scanStructure(node);

    }

    @Override
    public void endVisit(ASTEMACompilationUnit node){
        Log.info("MCNNSTC","END_MCNNSTC_COMP");
    }
}
