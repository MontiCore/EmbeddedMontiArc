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
import de.monticore.lang.monticar.emadl._visitor.NetworkVisitor;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.ArchitectureNode;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.ComposedNetworkScanner;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.ArchitectureNodeScanner;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;

public class NetworkProcessor extends CommonSymbolTableCreator implements NetworkVisitor {


    private NetworkVisitor realThis = this;
    private ArchitectureNodeScanner architectureNodeScanner = null;
    private ArrayList<ArchitectureNode> archNodes = null;

    private String composedNetworksFilePath = "";


    public NetworkProcessor(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope, ArrayList<ArchitectureNode> archNodes, String composedNetworksFilePath) {
        super(resolvingConfig, enclosingScope);
        this.initSuperSTC();
        this.initNetworkProcessor(archNodes, composedNetworksFilePath);
    }

    public NetworkProcessor(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack, ArrayList<ArchitectureNode> archNodes, String composedNetworksFilePath) {
        super(resolvingConfig, scopeStack);
        this.initSuperSTC();
        this.initNetworkProcessor(archNodes, composedNetworksFilePath);
    }

    public void initSuperSTC(){

    }

    public void initNetworkProcessor(ArrayList<ArchitectureNode> archNodes, String composedNetworksFilePath){
        architectureNodeScanner = new ArchitectureNodeScanner(archNodes);
        this.archNodes = archNodes;
        this.composedNetworksFilePath = composedNetworksFilePath;
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

    public NetworkVisitor getRealThis() {
        return this.realThis;
    }

    public void setRealThis (NetworkVisitor rt) {
        if (this.realThis != rt) {
            this.realThis =  rt;
        }
    }

    public void setRealThis (EMADLVisitor v) {
        if (v instanceof NetworkVisitor){
            this.setRealThis((NetworkVisitor) v);
        }
    }

    @Override
    public void visit(ASTNode node) {

    }

    @Override
    public void endVisit(ASTNode node){
        architectureNodeScanner.scanForArchitectureNodes(node);
    }

    @Override
    public void visit(ASTEMACompilationUnit node){

    }

    @Override
    public void endVisit(ASTEMACompilationUnit node){
        ComposedNetworkScanner composedNetworkScanner = new ComposedNetworkScanner(archNodes, this.composedNetworksFilePath);
        composedNetworkScanner.checkAndProcessComponentOnMatch(node);
    }
}
