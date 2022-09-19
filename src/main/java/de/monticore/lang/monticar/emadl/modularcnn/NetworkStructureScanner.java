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
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;
import de.monticore.lang.monticar.emadl._ast.EMADLMill;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;


public class NetworkStructureScanner /*implements EMADLVisitor*/ {
    ArrayList<ASTArchitecture> architecturesNodes = new ArrayList<>();

    public NetworkStructureScanner() {

    }

    public ArrayList<ASTArchitecture> getArchitecturesNodes() {
        return architecturesNodes;
    }

    public ArtifactScope getNextArtifactScopeUp(Scope scope){
        if (scope == null) return null;
        else if (scope instanceof ArtifactScope) return (ArtifactScope) scope;
        else if (scope.getEnclosingScope().isPresent()) return getNextArtifactScopeUp(scope.getEnclosingScope().get());
        return null;
    }

    public GlobalScope getNextGlobalScopeUp(Scope scope){
        if (scope == null) return null;
        else if (scope instanceof GlobalScope) return (GlobalScope) scope;
        else if (scope.getEnclosingScope().isPresent()) return getNextGlobalScopeUp(scope.getEnclosingScope().get());
        return null;
    }

    public CommonScope getNextCommonScopeUp(Scope scope){
        if (scope == null) return null;
        else if (scope instanceof CommonScope) return (CommonScope) scope;
        else if (scope.getEnclosingScope().isPresent()) return getNextCommonScopeUp(scope.getEnclosingScope().get());
        return null;
    }

    public void scanForArchitectureNodes(ASTNode node){
            Log.info("NSS","NSS_ASTNODE");
            Log.info(node.toString(),"NSS_ASTNODE");
       // EMAComponentInstanceSymbolCreator instanceSymbolCreator = new EMAComponentInstanceSymbolCreator();
        if (node instanceof ASTArchitecture){
            Log.info("ARCH_FOUND","NSS_ASTNODE");
            if (!node.getEnclosingScopeOpt().isPresent() || !node.getSymbolOpt().isPresent()) return;
            ArtifactScope artifactScope = getNextArtifactScopeUp(node.getEnclosingScope());
            GlobalScope globalScope = getNextGlobalScopeUp(node.getEnclosingScope());
            Symbol symbol = node.getSymbolOpt().get();


            //Symbol symbol =


            if (artifactScope == null || globalScope == null || symbol == null || !artifactScope.getName().isPresent() || symbol.getKind() == null ) return;

            Log.info("Symbol:"+ symbol.toString(),"NSS_ASTNODE_NEW_SYMBOl");
            //artifactScope.getName().get();
            String name = artifactScope.getName().get();
            ArrayList<Collection<Symbol>> symbolCollectionArrayList = artifactScope.getLocalSymbols().values().stream().collect(Collectors.toCollection(ArrayList::new));
            if (symbolCollectionArrayList.size() == 0) {
                Log.info("Empty","NSS_ASTNODE_NEW_SYMBOl");
            }
            SymbolKind k = null;
            for (Collection<Symbol> c: symbolCollectionArrayList   ) {
                ArrayList<Symbol> symbolArrayList = c.stream().collect(Collectors.toCollection(ArrayList::new));
                Log.info("Empty","NSS_ASTNODE_NEW_SYMBOl");
                for (Symbol s: symbolArrayList){

                    k = s.getKind();
                    break;
                }
            }




            //Optional<Symbol> optArtitfactSymbol =
            SymbolKind kind = symbol.getKind();
            Optional<Symbol> optNewSym = globalScope.resolve(name, kind);
            Log.info(name + " " + kind.getName(),"NSS_ASTNODE_NEW_SYMBOl");
            if (!optNewSym.isPresent()){
                Log.info("New Symbol not present!" ,"NSS_ASTNODE_NEW_SYMBOl");
                return;
            }

            Symbol newSym = optNewSym.get();

            Log.info("New Symbol:" + newSym.toString(),"NSS_ASTNODE_NEW_SYMBOl");

            architecturesNodes.add((ASTArchitecture) node);
        }



        //instanceSymbolCreator.createInstances();
    }

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
