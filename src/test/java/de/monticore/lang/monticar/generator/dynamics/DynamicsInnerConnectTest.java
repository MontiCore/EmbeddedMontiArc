/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class DynamicsInnerConnectTest extends AbstractSymtabTest {

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
    public void Test_01_Basic() throws IOException {
        test("connectionFromInner.test1", "./target/generated-sources-cpp/dynamics/connectFromInner/test01");
    }

    @Test
    public void Test_02_Basic() throws IOException {
        test("connectionFromInner.test2", "./target/generated-sources-cpp/dynamics/connectFromInner/test02");
    }

    @Test
    public void Test_03_ModelEasy() throws IOException {

        test("connectionFromInner.test3.fAS", "./target/generated-sources-cpp/dynamics/connectFromInner/test03");
    }

    @Test
    public void Test_04_Basic2Level() throws IOException {
        test("connectionFromInner.test4", "./target/generated-sources-cpp/dynamics/connectFromInner/test04");
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
