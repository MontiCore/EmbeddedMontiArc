/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureKind;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
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

        //Log.info("INIT","MCNNSTC_INIT_ENCLOSING-SCOPE");
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
            //Log.info("MCNNSTC Set Real this EMADLVisitor pass","MCNNSTC_SET_REAL_THIS_INSTANCE");
            this.setRealThis((ModularNetworkVisitor) v);
        } else{
            //Log.info("MCNNSTC Set Real this EMADLVisitor fail","MCNNSTC_SET_REAL_THIS_INSTANCE");
        }

    }

    @Override
    public void traverse(ASTNode node){
        //Log.info("MCNNSTC","TRAVERSE_MCNNSTC");
    }

    @Override
    public void handle(ASTNode node) {
        //Log.info("MCNNSTC","HANDLE_MCNNSTC");
    }

    @Override
    public void visit(ASTNode node) {
        //Log.info("MCNNSTC","VISIT_MCNNSTC_ASTNODE");
        //Log.info(node.toString(),"VISIT_MCNNSTC");

    }

    @Override
    public void endVisit(ASTNode node){
        if (node instanceof ASTEMADLNode) Log.info("EMADLNODE","END_MCNNSTC_AST_EMADLNODE");

        nss.scanForArchitectureNodes(node);

        if (node.get_SourcePositionStartOpt().isPresent() && node.get_SourcePositionStartOpt().get().getFileName().get().contains("MainB.emadl")){
            Log.info("Check node:" + node.toString(),"END_MCNNSTC_ASTNODE");
        }
       Log.info("MCNNSTC","END_MCNNSTC_ASTNODE");

    }


    @Override
    public void visit(ASTEMACompilationUnit node){

        //Log.info("MCNNSTC","VISIT_MCNNSTC_COMP");
        //Log.info("Node to String: " + node.toString(),"VISIT_MCNNSTC_COMP");

    }

    @Override
    public void endVisit(ASTEMACompilationUnit node){
        if (node instanceof ASTEMADLNode) Log.info("EMADLNODE","END_MCNNSTC_COMP_EMADLNODE");

        if (node.get_SourcePositionStartOpt().isPresent() && node.get_SourcePositionStartOpt().get().getFileName().get().contains("NetworkC.emadl")){
            Log.info("Check node:" + node.toString(),"END_MCNNSTC_ASTNODE");
            if (ArchitectureKind.KIND.isKindOf(node.getComponent().getSymbolOpt().get().getKind())){
                Log.info("ARCH_SYMBOL","END_MCNNSTC_ASTNODE");
            }
        }

        Log.info("MCNNSTC","END_MCNNSTC_COMP");
        //Log.info("Node: " + node.toString(),"END_MCNNSTC_COMP");
        CNNComposer cnnComposer = new CNNComposer(archNodes);

        if (cnnComposer.nodeIsComposedCNN(node) && cnnComposer.checkNotNullAndValid(node)){
            this.removeCurrentScope();
            cnnComposer.checkAndTransformComponentOnMatch(node);
            //MutableScope newScope = getFirstCreatedScope();
            //Log.info("NewScope: " + newScope.toString(),"END_MCNNSTC_COMP_DONE");
        }

        Log.info("DONE","END_MCNNSTC_COMP_DONE");
    }
}
