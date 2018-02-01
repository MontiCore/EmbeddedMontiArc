package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class ConfigPortTest extends AbstractSymtabTest{

    @Test
    public void testConfigPort() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol comp = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.adaptableParameterInstance",ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        ExpandedComponentInstanceSymbol subInst1 = comp.getSubComponent("adaptableParameter").orElse(null);
        assertNotNull(subInst1);

        PortSymbol configPort = subInst1.getIncomingPort("param1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/configPort/");
        List<File> files = generatorCPP.generateFiles(comp, symtab);




    }
}
