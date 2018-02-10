package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class CMakeMasterGenerator extends MasterGenerator {

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver, String targetPath) throws IOException {
        long notInSubfolders = getGeneratorImpls().stream()
                .map(this::getImplSubfolder)
                .filter(Objects::isNull)
                .count();

        if (notInSubfolders > 0) {
            Log.error("All generators must have subfolders!");
            return new ArrayList<>();
        }

        List<File> res = super.generate(componentInstanceSymbol, taggingResolver, targetPath);
        res.add(generateCMake(targetPath));
        return res;
    }

    private File generateCMake(String targetPath) throws IOException {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");
        StringBuilder content = new StringBuilder();
        content.append("cmake_minimum_required(VERSION 3.5)\n");
        //TODO setProjectName?
        content.append("project (default)\n");
        content.append("set (CMAKE_CXX_STANDARD 11)\n");

        getGeneratorImpls().stream()
                .map(this::getImplSubfolder)
                .forEach(subfolder -> content.append("add_subdirectory(" + subfolder + ")\n"));


        fileContent.setFileContent(content.toString());

        return FileHelper.generateFile(targetPath, fileContent);
    }


}
