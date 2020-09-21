/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.cocos.EmbeddedMontiArcDynamicCoCos;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEmbeddedMontiArcDynamicNode;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._parser.EmbeddedMontiArcDynamicParser;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicLanguage;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Optional;

@Ignore
public class DevTest extends AbstractTest {




    @Test
    public void testRESOLVERTEST2() throws Exception {
//        Scope symTab = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic");
//
//        Optional<EMADynamicComponentSymbol> comp = Optional.empty();
//        try {
//            comp = symTab.resolve(
//                    "parser.Test2", EMADynamicComponentSymbol.KIND);
//
//
//            if(comp.isPresent()) {
//
//                /*Collection<EMADynamicEventHandlerSymbol> col = comp.get().getEventHandler();
//                for (EMADynamicEventHandlerSymbol eh : col) {
//                    System.out.println(eh.getCondition().getTextualRepresentation());
//                }*/
//
//                EmbeddedMontiArcDynamicCoCos.createChecker().checkAll((ASTEmbeddedMontiArcDynamicNode)comp.get().getAstNode().get());
//
//                System.out.println(comp.get().toString());
//
//            }else{
//                System.out.println("comp not present!");
//            }
//
//
//        }catch (Exception ex){
//            ex.printStackTrace();
//        }
    }

    @Test
    public void testEventWithPort() throws IOException {
        /*EmbeddedMontiArcDynamicParser ep = new EmbeddedMontiArcDynamicParser();
        Optional<ASTEMACompilationUnit> parsedEvent = ep.parse("./src/test/resources/test/embeddedmontiarcdynamic/event/handling/test1/Not.emad");
        if(ep.hasErrors() || !parsedEvent.isPresent()){
            Log.error("There were unexpected parser errors");
        }else{
            Log.getFindings().clear();
        }*/
        Log.enableFailQuick(true);
        Scope symTab = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/event");
        Optional<EMADynamicComponentSymbol> comp = Optional.empty();
        try {
            comp = symTab.resolve("handling.test1.Not", EMADynamicComponentSymbol.KIND);
//            comp = symTab.resolve("parser.Test4", EMADynamicComponentSymbol.KIND);
            if (comp.isPresent()) {
                System.out.println(comp.get().toString());
                 
            } else {
                System.out.println("component could not be resolved!");
            }
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }

    @Test
    public void testEventWithPort2() throws IOException {
        /*EmbeddedMontiArcDynamicParser ep = new EmbeddedMontiArcDynamicParser();
        Optional<ASTEMACompilationUnit> parsedEvent = ep.parse("./src/test/resources/test/embeddedmontiarcdynamic/event/handling/test1/Not.emad");
        if(ep.hasErrors() || !parsedEvent.isPresent()){
            Log.error("There were unexpected parser errors");
        }else{
            Log.getFindings().clear();
        }*/
        Log.enableFailQuick(true);
        Scope symTab = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/event");
        Optional<EMADynamicComponentInstanceSymbol> comp = Optional.empty();
        try {
            comp = symTab.resolve("handling.test1.not", EMADynamicComponentInstanceSymbol.KIND);
//            comp = symTab.resolve("parser.Test4", EMADynamicComponentSymbol.KIND);
            if (comp.isPresent()) {
                System.out.println(comp.get().toString());
                 
            } else {
                System.out.println("component could not be resolved!");
            }
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }

    @Test
    public void testEMA(){
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
//        fam.addModelingLanguage(new EmbeddedMontiArcDynamicLanguage());
//        fam.addModelingLanguage(new EventLanguage());
        fam.addModelingLanguage(new EmbeddedMontiArcLanguage());

        final ModelPath mp = new ModelPath();
        mp.addEntry(Paths.get("./src/test/resources/test/embeddedmontiarcdynamic/ema"));
        GlobalScope scope = new GlobalScope(mp, fam);

        try {
            Optional<EMAComponentInstanceSymbol> comp = scope.resolve("test.test1", EMAComponentInstanceSymbol.KIND);
//            comp = symTab.resolve("parser.Test4", EMADynamicComponentSymbol.KIND);
            if (comp.isPresent()) {
                System.out.println(comp.get().toString());
                 
            } else {
                System.out.println("component could not be resolved!");
            }
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }

    @Test
    public void testEventWithPort3() throws IOException {

        Log.enableFailQuick(true);
        Scope symTab = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic");
        Optional<EMADynamicComponentSymbol> comp = Optional.empty();
        try {
            comp = symTab.resolve("parser.Test1", EMADynamicComponentSymbol.KIND);
//            comp = symTab.resolve("parser.Test4", EMADynamicComponentSymbol.KIND);
            if (comp.isPresent()) {
                System.out.println(comp.get().toString());
                 
            } else {
                System.out.println("component could not be resolved!");
            }
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }


    @Test
    public void testNotAdapter()  {
        Log.enableFailQuick(true);
        Scope symTab = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic");
        Optional<EMADynamicComponentSymbol> comp = Optional.empty();
        try {
            comp = symTab.resolve("easy.NotAdapter", EMADynamicComponentSymbol.KIND);
//            comp = symTab.resolve("parser.Test4", EMADynamicComponentSymbol.KIND);
            if (comp.isPresent()) {
                System.out.println(comp.get().toString());
                 
            } else {
                System.out.println("component could not be resolved!");
            }
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }

    @Test
    public void testNotAdapterInstance()  {
        Log.enableFailQuick(true);
        Scope symTab = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic");
        Optional<EMADynamicComponentInstanceSymbol> comp = Optional.empty();
        try {
            comp = symTab.resolve("easy.notAdapter", EMADynamicComponentInstanceSymbol.KIND);
//            comp = symTab.resolve("parser.Test4", EMADynamicComponentSymbol.KIND);
            if (comp.isPresent()) {
                System.out.println(comp.get().toString());
//                 
            } else {
                System.out.println("component could not be resolved!");
            }
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }
}
