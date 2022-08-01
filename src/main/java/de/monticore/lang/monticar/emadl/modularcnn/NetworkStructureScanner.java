/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;


import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Optional;


public class NetworkStructureScanner /*implements EMADLVisitor*/ {

    public NetworkStructureScanner() {

    }

    public void scanStructure(ASTNode node){
            Log.info("NSS","NSS_ASTNODE");
        EMAComponentInstanceSymbolCreator instanceSymbolCreator = new EMAComponentInstanceSymbolCreator();


        //instanceSymbolCreator.createInstances();
    }

    public void scanStructure(ASTEMACompilationUnit node){
        Log.info("SCAN_STRUCTURE","NSS_SCAN_COMPS");
        ASTComponent rootComp = node.getComponent();
        List<ASTSubComponent> subComponentList = rootComp.getSubComponents();
        Log.info("LOOP Start","NSS_SCAN_COMPS");
        for (ASTSubComponent subComp : subComponentList){
            Log.info(subComp.toString(),"NSS_SCAN_COMPS_LOOP");
            Optional<? extends Symbol> sym = subComp.getSymbolOpt();
            if (sym.isPresent()){
                Log.info("Symbol is null","NSS_SCAN_COMPS_LOOP");
                Log.info(subComp.getSymbolOpt().get().getName(),"NSS_SCAN_COMPS_LOOP");
                Log.info(sym.toString(),"NSS_SCAN_COMPS_LOOP");
                //continue;
            } else{
                List<ASTSubComponentInstance> instances = subComp.getInstancesList();
                for (ASTSubComponentInstance i : instances){
                    Log.info(i.getName(),"NSS_SCAN_COMPS_LOOP");
                }
            }



            //subComp.getSymbol().get

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
