package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.*;
import de.monticore.lang.monticar.generator.middleware.impls.GeneratorImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RclCppGenImpl;
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
    public List<File> generate(EMAComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        Map<EMAComponentInstanceSymbol, GeneratorImpl> generatorMap = new HashMap<>();

        fixComponentInstance(componentInstanceSymbol);

        List<EMAComponentInstanceSymbol> clusterSubcomponents = ClusterHelper.getClusterSubcomponents(componentInstanceSymbol);
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

        for (EMAComponentInstanceSymbol comp : generatorMap.keySet()) {
            files.addAll(generatorMap.get(comp).generate(comp, taggingResolver));
            //add empty generator to subDirs so that CMakeLists.txt will be generated correctly
            subDirs.add(NameHelper.getNameTargetLanguage(comp.getFullName()));
        }

        files.add(generateCMake(componentInstanceSymbol));
        return files;
    }

    private GeneratorImpl createFullGenerator(String subdir) {
        MiddlewareGenerator res = new MiddlewareGenerator();
        res.setGenerationTargetPath(generationTargetPath + (subdir.endsWith("/") ? subdir : subdir + "/"));

        this.getGeneratorImpls().forEach(gen -> res.add(gen, this.getImplSubdir(gen)));

        return res;
    }

    private void fixComponentInstance(EMAComponentInstanceSymbol componentInstanceSymbol) {
        RosHelper.fixRosConnectionSymbols(componentInstanceSymbol, this.getGeneratorImpls().stream().anyMatch(g -> g instanceof RclCppGenImpl));
    }

    @Override
    protected File generateCMake(EMAComponentInstanceSymbol componentInstanceSymbol) throws IOException {
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
