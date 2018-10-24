package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.*;
import de.monticore.lang.monticar.generator.middleware.impls.GeneratorImpl;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class DistributedTargetGenerator extends CMakeGenerator {
    private Set<String> subDirs = new HashSet<>();


    public DistributedTargetGenerator() {
    }

    @Override
    public void setGenerationTargetPath(String path) {
        super.setGenerationTargetPath(path);
    }

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        Map<ExpandedComponentInstanceSymbol, GeneratorImpl> generatorMap = new HashMap<>();

        fixComponentInstance(componentInstanceSymbol);

        List<ExpandedComponentInstanceSymbol> clusterSubcomponents = ClusterHelper.getClusterSubcomponents(componentInstanceSymbol);
        if (clusterSubcomponents.size() > 0) {
            clusterSubcomponents.forEach(clusterECIS -> {
                String nameTargetLanguage = NameHelper.getNameTargetLanguage(clusterECIS.getFullName());
                generatorMap.put(clusterECIS, createFullGenerator(nameTargetLanguage));
            });
        } else {
            String nameTargetLanguage = NameHelper.getNameTargetLanguage(componentInstanceSymbol.getFullName());
            generatorMap.put(componentInstanceSymbol, createFullGenerator(nameTargetLanguage));
        }

        List<File> files = new ArrayList<>();

        for (ExpandedComponentInstanceSymbol comp : generatorMap.keySet()) {
            files.addAll(generatorMap.get(comp).generate(comp, taggingResolver));
            //add empty generator to subDirs so that CMakeLists.txt will be generated correctly
            subDirs.add(NameHelper.getNameTargetLanguage(comp.getFullName()));
        }

        //generate rosMsg CMake iff a .msg file was generated
        if(files.stream().anyMatch(f -> f.getName().endsWith(".msg"))){
            subDirs.add("rosMsg");
            files.add(generateRosMsgGen());
        }

        files.add(generateCMake(componentInstanceSymbol));
        return files;
    }

    private File generateRosMsgGen() throws IOException {
        File file = new File(generationTargetPath + "rosMsg/CMakeLists.txt");
        FileUtils.write(file, TemplateHelper.getStruct_msgsCmakeTemplate());
        return file;
    }

    private GeneratorImpl createFullGenerator(String subdir) {
        MiddlewareGenerator res = new MiddlewareGenerator();
        res.setGenerationTargetPath(generationTargetPath + (subdir.endsWith("/") ? subdir : subdir + "/"));

        this.getGeneratorImpls().forEach(gen -> res.add(gen, this.getImplSubdir(gen)));

        return res;
    }

    private void fixComponentInstance(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        RosHelper.fixRosConnectionSymbols(componentInstanceSymbol);
    }

    @Override
    protected File generateCMake(ExpandedComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");
        StringBuilder content = new StringBuilder();
        content.append("cmake_minimum_required(VERSION 3.5)\n");
        //TODO setProjectName?
        content.append("project (default)\n");
        content.append("set (CMAKE_CXX_STANDARD 11)\n");

        subDirs.stream().filter(dir -> dir.equals("rosMsg")).forEach(
                dir -> content.append("add_subdirectory(" + dir + ")\n")
        );

        subDirs.stream().filter(dir -> !dir.equals("rosMsg")).forEach(
                dir -> content.append("add_subdirectory(" + dir + ")\n")
        );

        fileContent.setFileContent(content.toString());

        return FileHelper.generateFile(generationTargetPath, fileContent);
    }

}
