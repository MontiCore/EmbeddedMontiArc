package de.monticore.lang.monticar.generator.master.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.master.helpers.TemplateHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

//TODO: make GeneratorCpp implement GeneratorImpl directly!
public class CPPGenImpl implements GeneratorImpl {
    private String generationTargetPath;


    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        List<File> files = new ArrayList<>();

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath(generationTargetPath);
        generatorCPP.useArmadilloBackend();
        files.add(generatorCPP.generateFile(generateCMake(componentInstanceSymbol)));
        files.addAll(generatorCPP.generateFiles(componentInstanceSymbol, taggingResolver));

        return files;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path;
    }

    private FileContent generateCMake(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        FileContent cmake = new FileContent();
        cmake.setFileName("CMakeLists.txt");
        String name = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        cmake.setFileContent(TemplateHelper.cmakeCppTemplate.replace("${compName}", name));
        return cmake;
    }
}
