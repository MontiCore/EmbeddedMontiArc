/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mathopt;

//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
//import de.monticore.lang.monticar.generator.cpp.EMAMOpt2CPPSymbolTableHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class CarModelGenerationTest extends AbstractSymtabTest {

//    private static TaggingResolver symtab = EMAMOpt2CPPSymbolTableHelper.getInstance().createSymTabAndTaggingResolver("src/test/resources");

    protected static List<File> doGenerateModel(String fullModelName) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/mathopt");
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullModelName, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/mathopt/carmodel/" + fullModelName.substring(fullModelName.lastIndexOf(".") + 1, fullModelName.length()));
        return generatorCPP.generateFiles(componentInstanceSymbol, symtab);
    }

    @Test
    public void carModelInstanceTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.carmodel.carModelInstanceTest");
    }
}
