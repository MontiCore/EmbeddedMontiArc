/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dynamics;

import alice.tuprolog.Int;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertNotNull;


public class DynamicsPortFreeTest extends AbstractSymtabTest {

    protected String path(){
        return "src/test/resources/dynamics";
    }

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void Test_00_BasicFree() throws IOException {
        test("free.free", "./target/generated-sources-cpp/dynamics/free/test00");
    }

    @Test
    public void Test_01_SetFree() throws IOException {
        test("free.freeSetOnValue", "./target/generated-sources-cpp/dynamics/free/test01");
    }

    @Test
    public void Test_02_OutFree() throws IOException {
        test("free.outerFree", "./target/generated-sources-cpp/dynamics/free/test02");
    }

    @Test
    public void Test_03_OutFree2() throws IOException {
        test("free.outerFree2", "./target/generated-sources-cpp/dynamics/free/test03");
    }

    protected void test(String instName, String target){
        try {
            TaggingResolver symtab = createSymTabAndTaggingResolver(path());

            EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(instName, EMAComponentInstanceSymbol.KIND).orElse(null);
            assertNotNull(componentSymbol);
            GeneratorCPP generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
            generatorCPP.setGenerationTargetPath(target);
            List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        }catch (IOException ex){
            ex.printStackTrace();
            Log.error(ex.getMessage());
        }
    }
}
