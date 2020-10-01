/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.nio.file.Paths;
import java.util.List;

/**
 */
public class AutomaticStreamTestGenerator extends AbstractSymtab {

    public void generateTests(String fullComponentInstanceName, String basePath, String targetPath, String testNamePostFix, int amountTickValues) throws Exception {
        TaggingResolver symtab = createSymTabAndTaggingResolver(basePath);
        EMAComponentInstanceSymbol componentSymbol = null;
        if (fullComponentInstanceName.length() > 0) {
            componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullComponentInstanceName, EMAComponentInstanceSymbol.KIND).orElse(null);
        }
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration(testNamePostFix,amountTickValues);
        generatorCPP.setGenerationTargetPath(targetPath);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get(basePath));
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);

    }
}
