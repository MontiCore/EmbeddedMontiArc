package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.FileHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class CMakeGenerator extends StarBridgeGenerator {

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        long notInSubdirs = getGeneratorImpls().stream()
                .map(this::getImplSubdir)
                .filter(Objects::isNull)
                .count();

        if (notInSubdirs > 0) {
            Log.error("All generators must have subdirs!");
            return new ArrayList<>();
        }

        List<File> res = super.generate(componentInstanceSymbol, taggingResolver);
        res.add(generateCMake(componentInstanceSymbol));
        return res;
    }

    protected File generateCMake(ExpandedComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");
        StringBuilder content = new StringBuilder();
        content.append("cmake_minimum_required(VERSION 3.5)\n");
        //TODO setProjectName?
        content.append("project (default)\n");
        content.append("set (CMAKE_CXX_STANDARD 11)\n");

        getGeneratorImpls().stream()
                .filter(gen -> gen.willAccept(componentInstanceSymbol))
                .map(this::getImplSubdir)
                .sorted()
                .forEach(subdir -> content.append("add_subdirectory(" + subdir + ")\n"));


        fileContent.setFileContent(content.toString());

        return FileHelper.generateFile(generationTargetPath, fileContent);
    }


}
