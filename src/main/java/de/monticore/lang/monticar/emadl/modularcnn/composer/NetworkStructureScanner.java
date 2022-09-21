/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;


import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


public class NetworkStructureScanner /*implements EMADLVisitor*/ {
    ArrayList<ArchitectureNode> architecturesNodes = null;

    public NetworkStructureScanner(ArrayList<ArchitectureNode> currentNodes) {
        this.architecturesNodes = currentNodes;

    }

    /*
    public ArrayList<ASTArchitecture> getArchitecturesNodes() {
        return architecturesNodes;
    }
    */



    public void scanForArchitectureNodes(ASTNode node){
        if (node instanceof ASTArchitecture){

           if ((architecturesNodes.size() == 0)) architecturesNodes.add(new ArchitectureNode((ASTArchitecture) node));

           for (ArchitectureNode architectureNode: architecturesNodes){
               if (architectureNode.getOriginalNode().equals((ASTArchitecture) node)) return;
           }

            architecturesNodes.add(new ArchitectureNode((ASTArchitecture) node));
        }
    }

    /*public void scanForArchitectureNodes(ASTEMACompilationUnit node){
        Log.info("NSS","NSS_CU");
        Log.info(node.toString(),"NSS_CU");

    }*/

    public void scanStructure(ASTEMADLNode node){
        Log.info("NSS","NSS_ASTEMADLNODE");
        //EMAComponentInstanceSymbolCreator instanceSymbolCreator = new EMAComponentInstanceSymbolCreator();



        //instanceSymbolCreator.createInstances();
    }



    public void scanStructure(ASTEMACompilationUnit node){
        ArrayList<Symbol> symbols = new ArrayList<>();
        ArrayList<ASTNode> nodes = new ArrayList<>();
        Log.info("SCAN_STRUCTURE","NSS_SCAN_COMPS");

        ASTComponent rootComp = node.getComponent();


        if (rootComp.getSymbolOpt().isPresent()) Log.info("MainComp: " + rootComp.getSymbolOpt().get().getName(),"NSS_SCAN_COMPS");
        List<ASTSubComponent> subComponentList = rootComp.getSubComponents();
        //Log.info("LOOP Start","NSS_SCAN_COMPS");
        for (ASTSubComponent subComp : subComponentList){
            Log.info(subComp.toString(),"NSS_SCAN_COMPS_LOOP_1");
            Optional<? extends Symbol> sym = subComp.getSymbolOpt();
            /*if (sym.isPresent()){
                Log.info("Symbol is null","NSS_SCAN_COMPS_LOOP_1");
                Log.info(subComp.getSymbolOpt().get().getName(),"NSS_SCAN_COMPS_LOOP_1");
                Log.info(sym.toString(),"NSS_SCAN_COMPS_LOOP_1");
                symbols.add(sym.get());
                if (sym.get().getAstNode().isPresent()) nodes.add(sym.get().getAstNode().get());

                //continue;
            } else{*/
                List<ASTSubComponentInstance> instances = subComp.getInstancesList();
                for (ASTSubComponentInstance i : instances){
                    Log.info(i.getName(),"NSS_SCAN_COMPS_LOOP_2");
                   // i.getSpannedScopeOpt().
                    if (i.getSymbolOpt().isPresent()) symbols.add(i.getSymbolOpt().get());
                    if (i.getSymbolOpt().isPresent() && i.getSymbolOpt().get().getAstNode().isPresent()) nodes.add(i.getSymbolOpt().get().getAstNode().get());
                //}
            }



            //subComp.getSymbol().get
            Log.info("breaker","BREAKER");
        }


        if (nodes.size() > 0 || symbols.size() > 0) {
            Log.info("Symbols size:" + symbols.size(),"NSS_SCAN_SIZES");
            Log.info("Nodes size: " + nodes.size(),"NSS_SCAN_SIZES");

        }



        /*
        Log.info("NSS","NSS_ASTEMACOMP");
        EMAComponentInstanceSymbolCreator instanceSymbolCreator = new EMAComponentInstanceSymbolCreator();
        Optional<? extends Symbol> sym = node.getComponent().getSymbolOpt();
        if (sym.isPresent()){
            instanceSymbolCreator.createInstances((EMAComponentSymbol)(Log.errorIfNull(sym.orElse(null))),node.getComponent().getSymbolOpt().get().getName());

        }
        Log.info("NSS","NSS_ASTEMACOMP");
        //instanceSymbolCreator.createInstances();
        */
    }

}
