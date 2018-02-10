package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CPPImpl implements GeneratorImpl {
    private String generationTargetPath;
    private String cmakeTemplate =
            "cmake_minimum_required(VERSION 3.5)\n" +
                    "project(${compName} LANGUAGES CXX)\n" +
                    "add_library(${compName} ${compName}.h)\n" +
                    "target_include_directories(${compName} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})\n" +
                    "set_target_properties(${compName} PROPERTIES LINKER_LANGUAGE CXX)\n" +
                    "export(TARGETS ${compName} FILE ${compName}.cmake)";

    //TODO: make GeneratorCpp implement GeneratorImpl directly!
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
        cmake.setFileContent(cmakeTemplate.replace("${compName}", name));
        return cmake;
    }
}
