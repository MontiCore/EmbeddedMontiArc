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



        //Log.info("INIT","MCNNSTC_INIT_SCOPE-STACK");
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


       //nss.scanForArchitectureNodes(node);
    }

    @Override
    public void endVisit(ASTNode node){

        nss.scanForArchitectureNodes(node);
        /*
        if (node instanceof  ASTEMADLNode){
            Log.info("HIT:" + node.toString(),"END_MCNNSTC_ASTNODE");
        }

        node.getSymbolOpt().ifPresent(symbol -> {
            if (symbol.getName().equals("Network")) {
                Log.info("HIT:" + symbol.getName(),"END_MCNNSTC_ASTNODE");
            }
        });


        ArrayList<ASTArchitecture> newArchNodes = nss.getArchitecturesNodes();
        Log.info("Size of ArchNode: " + newArchNodes.size(),"END_MCNNSTC_ASTNODE");

        for (int i=0;i< newArchNodes.size();i++) {
            if (!archNodes.contains(newArchNodes.get(i))){
                archNodes.add(newArchNodes.get(i));
            }

        }


        Log.info("MCNNSTC","END_MCNNSTC_ASTNODE");

         */
    }


    @Override
    public void visit(ASTEMACompilationUnit node){
        /*
        Log.info("MCNNSTC","VISIT_MCNNSTC_COMP");
        Log.info("Node to String: " + node.toString(),"VISIT_MCNNSTC_COMP");

        if (node.getComponent().getName().equals("Network")) {

            Log.info("Check out this node: " + node.toString(),"VISIT_MCNNSTC_COMP_MATCH");
        }
        */

        //NetworkStructureScanner nss = new NetworkStructureScanner();
        //nss.scanStructure(node);


    }

    @Override
    public void endVisit(ASTEMACompilationUnit node){
        Log.info("MCNNSTC","END_MCNNSTC_COMP");
        Log.info("Node: " + node.toString(),"END_MCNNSTC_COMP");
        CNNComposer cnnComposer = new CNNComposer(archNodes);
        cnnComposer.checkAndTransformComponentOnMatch(node);

        Log.info("DONE","END_MCNNSTC_COMP_DONE");

        /*
        if (node.getComponent().getName().equals("Net1")) {
            //cnnChecker.check(node.getComponent());
            Log.info("Check out node: " + node.toString(),"END_MCNNSTC_COMP_MATCH");
        } else if (node.getComponent().getName().equals("Net2")) {
            //cnnChecker.check(node.getComponent());
            Log.info("Check out node: " + node.toString(),"END_MCNNSTC_COMP_MATCH");

        } else if (node.getComponent().getName().equals("Network")) {
            //cnnChecker.check(node.getComponent());
            Log.info("Check out this node: " + node.toString(),"END_MCNNSTC_COMP_MATCH");

            List<? extends Scope> list =  node.getComponent().getSpannedScope().getSubScopes();

            for (int i=0;i<list.size();i++) {
                Symbol symbol = list.get(i).getSpanningSymbol().get();

                if (symbol.getKind().equals(ArchitectureSymbol.KIND)){
                    Log.info("KIND FOUND","END_MCNNSTC_COMP_MATCH_P");
                }
                else {
                    Log.info("N:" + symbol.getFullName(),"END_MCNNSTC_COMP_MATCH_N");
                    Log.info("KIND NOT FOUND","END_MCNNSTC_COMP_MATCH_N");
                }
            }


        }


        /*
        if (node.getComponent().getName().equals("Network")) {
            this.removeCurrentScope();
            Log.info("Check out this node: " + node.toString(),"END_MCNNSTC_COMP_MATCH");
        }
         */


       /* Deque<MutableScope> scopeStackRef = this.scopeStack;
        while (scopeStackRef.size() > 0 && scopeStackRef.iterator().hasNext() ){
            MutableScope currentScope = scopeStackRef.iterator().next();
            Optional<Symbol> sym = currentScope.resolve("InstanceTest.mainB", EMAComponentInstanceSymbol.KIND);
            // Optional<Symbol> sym = currentScope.resolve("", EMAComponentInstanceSymbol.KIND);
            if (sym.isPresent()){
                Log.info("Symbol found:" + sym.toString(),"END_MCNNSTC_COMP_SCOPE");
            }


            Log.info("Current Scope: "+ currentScope.toString(),"END_MCNNSTC_COMP_SCOPE");
        }*/



        //Optional<? extends Symbol> sym = node.getComponent().getSymbolOpt();
        //Log.info("endVisit of " + node.getComponent().getSymbolOpt().get().getFullName(), "END_MCNNSTC_COMP");


        
    }

    /*
    @Override
    public void visit(ASTEMADLNode node) {
        Log.info("MCNNSTC","VISIT_MCNNSTC_EMADL");
        Log.info(node.toString(),"VISIT_MCNNSTC");

        //NetworkStructureScanner nss = new NetworkStructureScanner();
        //nss.scanStructure(node);
    }

    @Override
    public void endVisit(ASTEMADLNode node){
        Log.info("MCNNSTC","END_MCNNSTC_EMADL");
        Log.info(node.toString(),"END_MCNNSTC_EMADL");
    }
    */

}
