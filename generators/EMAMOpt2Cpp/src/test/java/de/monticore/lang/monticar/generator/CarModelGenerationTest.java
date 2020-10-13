/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.EMAMOpt2CPPSymbolTableHelper;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class CarModelGenerationTest extends BasicGenerationTest {

    private static TaggingResolver symtab = EMAMOpt2CPPSymbolTableHelper.getInstance().createSymTabAndTaggingResolver("src/test/resources");

    protected static List<File> doGenerateModel(String fullModelName) throws IOException {
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullModelName, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorEMAMOpt2CPP generator = new GeneratorEMAMOpt2CPP();
        generator.setGenerateCMake(true);
        generator.setGenerationTargetPath("./target/generated-sources-cmake/" + fullModelName.substring(fullModelName.lastIndexOf(".") + 1, fullModelName.length()) + "/src/");
        return generator.generate(componentInstanceSymbol, symtab);
    }

    @Test
    public void carModelInstanceTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.carmodel.carModelInstanceTest");
    }
}
