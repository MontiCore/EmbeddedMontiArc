/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Ignore;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;


public class DynamicEventTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void Test_00_TestTrueEvent(){
        test("event.test00.test00", "./target/generated-sources-cpp/dynamics/event/test00");
    }


    protected void test(String instName, String target){
        try {
            TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

            EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(instName, EMAComponentInstanceSymbol.KIND).orElse(null);
            assertNotNull(componentSymbol);

            GeneratorCPP generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
            generatorCPP.setGenerationTargetPath(target);
//            generatorCPP.setUseThreadingOptimization(true);
            List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        }catch (IOException ex){
            ex.printStackTrace();
            Log.error(ex.getMessage());
        }
    }
}
