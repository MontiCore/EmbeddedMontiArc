package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class MasterGenerator implements GeneratorImpl {
    private Map<GeneratorImpl, String> generatorImpls = new HashMap<>();
    String generationTargetPath;

    public void add(GeneratorImpl generator, String subfolder) {
        generatorImpls.put(generator, subfolder);
    }

    public Set<GeneratorImpl> getGeneratorImpls() {
        return generatorImpls.keySet();
    }

    public String getImplSubfolder(GeneratorImpl generator) {
        return generatorImpls.get(generator);
    }

    public boolean remove(GeneratorImpl generator) {
        return generatorImpls.remove(generator) != null;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path.endsWith("/") ? path : path + "/";
    }

    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        List<File> result = new ArrayList<>();
        generatorImpls.entrySet()
                .forEach(entrySet -> {
                    String fullTargetPath = generationTargetPath;
                    if (entrySet.getValue() != null)
                        fullTargetPath = fullTargetPath + entrySet.getValue();

                    if (!fullTargetPath.endsWith("/"))
                        fullTargetPath = fullTargetPath + "/";

                    entrySet.getKey().setGenerationTargetPath(fullTargetPath);

                    try {
                        result.addAll(entrySet.getKey().generate(componentInstanceSymbol, taggingResolver));
                    } catch (IOException e) {
                        Log.error("IOException occurred!", e);
                    }
                });

        return result;
    }


}
