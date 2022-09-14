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
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureKind;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.lang.monticar.emadl._visitor.ModularNetworkVisitor;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;
import java.util.Optional;

public class ModularCNNSymbolTableCreator extends CommonSymbolTableCreator implements ModularNetworkVisitor {


    private ModularNetworkVisitor realThis = this;
    private NetworkStructureScanner nss = null;

    public ModularCNNSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        this.initSuperSTC();
        this.initNetworkStructureScanner();
        Log.info("INIT","MCNNSTC_INIT_ENCLOSING-SCOPE");

    }

    public ModularCNNSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        this.initSuperSTC();
        this.initNetworkStructureScanner();
        Log.info("INIT","MCNNSTC_INIT_SCOPE-STACK");
    }

    public void initSuperSTC(){

    }

    public void initNetworkStructureScanner(){
        nss = new NetworkStructureScanner();
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
        Log.info("MCNNSTC","VISIT_MCNNSTC_ASTNODE");
        Log.info(node.toString(),"VISIT_MCNNSTC");


        nss.scanForArchitectureNodes(node);
    }

    @Override
    public void endVisit(ASTNode node){
        ArrayList<ASTArchitecture> archNodes = nss.getArchitecturesNodes();
        Log.info("Size of ArchNode: " + archNodes.size(),"END_MCNNSTC_ASTNODE");


        Log.info("MCNNSTC","END_MCNNSTC_ASTNODE");
    }


    @Override
    public void visit(ASTEMACompilationUnit node){
        Log.info("MCNNSTC","VISIT_MCNNSTC_COMP");
        Log.info("Node to String: " + node.toString(),"VISIT_MCNNSTC_COMP");


        NetworkStructureScanner nss = new NetworkStructureScanner();
        nss.scanStructure(node);

    }

    @Override
    public void endVisit(ASTEMACompilationUnit node){
        Log.info("MCNNSTC","END_MCNNSTC_COMP");
        this.removeCurrentScope();

        Deque<MutableScope> scopeStackRef = this.scopeStack;
        while (scopeStackRef.size() > 0 && scopeStackRef.iterator().hasNext() ){
            MutableScope currentScope = scopeStackRef.iterator().next();
            Optional<Symbol> sym = currentScope.resolve("InstanceTest.mainB", EMAComponentInstanceSymbol.KIND);
            // Optional<Symbol> sym = currentScope.resolve("", EMAComponentInstanceSymbol.KIND);
            if (sym.isPresent()){
                Log.info("Symbol found:" + sym.toString(),"END_MCNNSTC_COMP_SCOPE");
            }


            Log.info("Current Scope: "+ currentScope.toString(),"END_MCNNSTC_COMP_SCOPE");
        }
        Log.info("DONE","END_MCNNSTC_COMP_DONE");
        //Optional<? extends Symbol> sym = node.getComponent().getSymbolOpt();
        //Log.info("endVisit of " + node.getComponent().getSymbolOpt().get().getFullName(), "END_MCNNSTC_COMP");


        
    }

    @Override
    public void visit(ASTEMADLNode node) {
        Log.info("MCNNSTC","VISIT_MCNNSTC_EMADL");
        Log.info(node.toString(),"VISIT_MCNNSTC");

        NetworkStructureScanner nss = new NetworkStructureScanner();
        nss.scanStructure(node);
    }

    @Override
    public void endVisit(ASTEMADLNode node){
        Log.info("MCNNSTC","END_MCNNSTC_EMADL");
    }
}
