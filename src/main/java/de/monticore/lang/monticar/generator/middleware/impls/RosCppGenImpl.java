package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.List;

//TODO: make GeneratorRosCpp implement GeneratorImpl
public class RosCppGenImpl implements GeneratorImpl {
    private String generationTargetPath;

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerateCMake(true);
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);
        return generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path;
    }
}
