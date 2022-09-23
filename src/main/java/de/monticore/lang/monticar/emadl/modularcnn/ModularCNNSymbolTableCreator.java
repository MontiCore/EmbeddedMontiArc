/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.lang.monticar.emadl._visitor.ModularNetworkVisitor;
import de.monticore.lang.monticar.emadl.modularcnn.composer.ArchitectureNode;
import de.monticore.lang.monticar.emadl.modularcnn.composer.CNNComposer;
import de.monticore.lang.monticar.emadl.modularcnn.composer.NetworkStructureScanner;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;

public class ModularCNNSymbolTableCreator extends CommonSymbolTableCreator implements ModularNetworkVisitor {


    private ModularNetworkVisitor realThis = this;
    private NetworkStructureScanner nss = null;
    private ArrayList<ArchitectureNode> archNodes = null;


    public ModularCNNSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope, ArrayList<ArchitectureNode> archNodes) {
        super(resolvingConfig, enclosingScope);
        this.initSuperSTC();
        this.initNetworkStructureScanner(archNodes);
    }

    public ModularCNNSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack, ArrayList<ArchitectureNode> archNodes) {
        super(resolvingConfig, scopeStack);
        this.initSuperSTC();
        this.initNetworkStructureScanner(archNodes);
    }

    public void initSuperSTC(){

    }

    public void initNetworkStructureScanner(ArrayList<ArchitectureNode> archNodes){
        nss = new NetworkStructureScanner(archNodes);
        this.archNodes = archNodes;
        //Log.info("Initialized Network Structure Scanner","NSS_INIT");
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
            this.setRealThis((ModularNetworkVisitor) v);
        }
    }

    @Override
    public void visit(ASTNode node) {

    }

    @Override
    public void endVisit(ASTNode node){
        nss.scanForArchitectureNodes(node);
    }

    @Override
    public void visit(ASTEMACompilationUnit node){

    }

    @Override
    public void endVisit(ASTEMACompilationUnit node){
        CNNComposer cnnComposer = new CNNComposer(archNodes);
        if (cnnComposer.checkNotNullAndValid(node)){
            cnnComposer.checkAndProcessComponentOnMatch(node);
        }
        Log.info("DONE","END_MCNNSTC_COMP_DONE");
    }
}
