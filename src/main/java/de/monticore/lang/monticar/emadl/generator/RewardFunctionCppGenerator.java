package de.monticore.lang.monticar.emadl.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.Optional;

/**
 *
 */
public class RewardFunctionCppGenerator implements RewardFunctionSourceGenerator {
    public RewardFunctionCppGenerator() {
    }

    @Override
    public void generate(String modelPath, String rootModel, String targetPath) {
        GeneratorEMAMOpt2CPP generator = new GeneratorEMAMOpt2CPP();
        generator.useArmadilloBackend();

        TaggingResolver taggingResolver = EMADLAbstractSymtab.createSymTabAndTaggingResolver(modelPath);
        Optional<EMAComponentInstanceSymbol> instanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve(rootModel, EMAComponentInstanceSymbol.KIND);

        if (!instanceSymbol.isPresent()) {
            Log.error("Generation of reward function is not possible: Cannot resolve component instance "
                + rootModel);
        }

        generator.setGenerationTargetPath(targetPath);

        try {
            generator.generate(instanceSymbol.get(), taggingResolver);
        } catch (IOException e) {
            Log.error("Generation of reward function is not possible: " + e.getMessage());
        }
    }
}
