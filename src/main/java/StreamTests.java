
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class StreamTests extends AbstractSymtab {
    private static final String EMAM_BASE = "./src/main/emam";
    private static final String STREAM_TEST_BASE = "./src/test/emam";
    private static final String TEST_MAIN = "./target/test-main";
    private static final String TEST_MAIN_CPP = TEST_MAIN+"/cpp";
    private static final String TEST_MAIN_EMAM_STREAM =  TEST_MAIN+"/emam";

    private static final Path EMAM_STREAM_JOIN_PATH = Paths.get(TEST_MAIN_EMAM_STREAM);


    protected TaggingResolver SymTab = null;

    public void Setup(){
        File srcDirEMAM = new File(EMAM_BASE);
        File srcDirSTREAM = new File(STREAM_TEST_BASE);
        File mainDir = new File(TEST_MAIN);
        File emamDir = new File(TEST_MAIN_EMAM_STREAM);
        try {
            FileUtils.deleteDirectory(mainDir);

            FileUtils.copyDirectory(srcDirEMAM, emamDir);
            FileUtils.copyDirectory(srcDirSTREAM, emamDir);

        }catch (IOException e){
            e.printStackTrace();
        }

        this.SymTab = createSymTabAndTaggingResolver(TEST_MAIN_EMAM_STREAM);
    }


    public void GenerateCPPforMainTest(){
        ExpandedComponentInstanceSymbol componentSymbol = SymTab.<ExpandedComponentInstanceSymbol>resolve(
                "mainTest.test",
                ExpandedComponentInstanceSymbol.KIND
        ).orElse(null);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(EMAM_STREAM_JOIN_PATH);

        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath(TEST_MAIN_CPP);
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateMainClass(true);
        generatorCPP.setCheckModelDir(false);

        try {
            generatorCPP.generateFiles(SymTab, componentSymbol, SymTab);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    /*
    public void CheckEachComponentHasStreamTest(){
        ComponentScanner cs = new ComponentScanner(EMAM_STREAM_JOIN_PATH, SymTab, "emam");
        Set<String> availableComponents = cs.scan();
        for (String s: availableComponents) {
            System.out.println(s);

        }

        StreamScanner scanner = new StreamScanner(EMAM_STREAM_JOIN_PATH, SymTab);
        Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams = new HashMap(scanner.scan());
        ComponentSymbol cos = null;
        for (ComponentSymbol compsym: availableStreams.keySet()) {
            System.out.println(compsym.getFullName());



            cos = compsym;
        }

        this.SymTab = createSymTabAndTaggingResolver(EMAM_STREAM_JOIN);
        ExpandedComponentInstanceSymbol componentSymbol = SymTab.<ExpandedComponentInstanceSymbol>resolve(
                "basicLib.BAnd.And",
                ExpandedComponentInstanceSymbol.KIND
        ).orElse(null);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(EMAM_STREAM_JOIN_PATH);

        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/test/basicLib/and2/");
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateMainClass(true);
        generatorCPP.setCheckModelDir(false);

        try {
            List<File> files = generatorCPP.generateFiles(SymTab, componentSymbol, SymTab);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public void RunStreamTests(){
        StreamScanner scanner = new StreamScanner(EMAM_STREAM_JOIN_PATH, SymTab);
        Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams = new HashMap(scanner.scan());
        ComponentSymbol cos = null;
        for (ComponentSymbol compsym: availableStreams.keySet()) {

            this.GenereateCPPFor(compsym.getFullName());
        }

        ComponentScanner cs = new ComponentScanner(EMAM_STREAM_JOIN_PATH, SymTab, "emam");
        for (String s : cs.scan()) {
            System.out.println(s);
        }

    }

    public void GenereateCPPFor(String compString){
        int idx = compString.lastIndexOf(".");
        compString = compString.substring(0,idx+1) + compString.substring(idx+1,idx+2).toLowerCase() + compString.substring(idx+2);
        ExpandedComponentInstanceSymbol componentSymbol = SymTab.<ExpandedComponentInstanceSymbol>resolve(
                compString,
                ExpandedComponentInstanceSymbol.KIND
        ).orElse(null);

        System.out.println("./target/cpp-test/"+compString.replaceAll(".", "/"));

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(EMAM_STREAM_JOIN_PATH);

        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/cpp-test/"+compString.replace(".", "/"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateMainClass(true);
        generatorCPP.setCheckModelDir(false);



        try {
            generatorCPP.generateFiles(SymTab, componentSymbol, SymTab);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }*/
}

