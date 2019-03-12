package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.cli.ClusteringParameters;
import de.monticore.lang.monticar.generator.middleware.cli.ResultChoosingStrategy;
import de.monticore.lang.monticar.generator.middleware.clustering.*;
import de.monticore.lang.monticar.generator.middleware.helpers.FileHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.NameHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.RosHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;
import de.monticore.lang.monticar.generator.middleware.impls.GeneratorImpl;
import de.monticore.lang.monticar.generator.middleware.impls.MiddlewareTagGenImpl;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class DistributedTargetGenerator extends CMakeGenerator {
    private boolean generateMiddlewareTags = false;
    private ClusteringResultList clusteringResults = new ClusteringResultList();

    public boolean isGenerateMiddlewareTags() {
        return generateMiddlewareTags;
    }

    public void setGenerateMiddlewareTags(boolean generateMiddlewareTags) {
        this.generateMiddlewareTags = generateMiddlewareTags;
    }

    private Set<String> subDirs = new HashSet<>();

    private ClusteringParameters clusteringParameters;

    public DistributedTargetGenerator() {
    }

    public ClusteringParameters getClusteringParameters() {
        return clusteringParameters;
    }

    public void setClusteringParameters(ClusteringParameters clusteringParameters) {
        this.clusteringParameters = clusteringParameters;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        super.setGenerationTargetPath(path);
    }

    @Override
    public List<File> generate(EMAComponentInstanceSymbol genComp, TaggingResolver taggingResolver) throws IOException {
        Map<EMAComponentInstanceSymbol, GeneratorImpl> generatorMap = new HashMap<>();

        EMAComponentInstanceSymbol componentInstanceSymbol = preprocessing(genComp);

        List<ExpandedComponentInstanceSymbol> clusterSubcomponents = ClusterFromTagsHelper.getClusterSubcomponents(componentInstanceSymbol);
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

        //generate rosMsg CMake iff a .msg file was generated
        if(files.stream().anyMatch(f -> f.getName().endsWith(".msg"))){
            subDirs.add("rosMsg");
            files.add(generateRosMsgGen());
        }

        if(generateMiddlewareTags){
            MiddlewareTagGenImpl middlewareTagGen = new MiddlewareTagGenImpl();
            middlewareTagGen.setGenerationTargetPath(generationTargetPath + "emam/");
            middlewareTagGen.setClusteringResults(clusteringResults);
            files.addAll(middlewareTagGen.generate(componentInstanceSymbol,taggingResolver));
        }

        files.add(generateCMake(componentInstanceSymbol));
        return files;
    }

    private EMAComponentInstanceSymbol preprocessing(EMAComponentInstanceSymbol genComp) {
        EMAComponentInstanceSymbol componentInstanceSymbol = genComp;
        if(clusteringParameters != null){
            //Flatten
            if(clusteringParameters.getFlatten()){
                if(clusteringParameters.getFlattenLevel().isPresent()){
                    Integer level = clusteringParameters.getFlattenLevel().get();
                    componentInstanceSymbol = FlattenArchitecture.flattenArchitecture(genComp, new HashMap<>(), level);
                }else {
                    componentInstanceSymbol = FlattenArchitecture.flattenArchitecture(genComp);
                }
                System.out.println("Subcomponents after flatten: " + componentInstanceSymbol.getSubComponents().size());
            }

            //Cluster
            if(clusteringParameters.getAlgorithmParameters().size() > 0) {
                clusteringResults = ClusteringResultList.fromParametersList(componentInstanceSymbol, clusteringParameters.getAlgorithmParameters());
                Optional<Integer> nOpt = clusteringParameters.getNumberOfClusters();
                for(ClusteringResult c : clusteringResults){
                    String prefix = nOpt.isPresent() && !c.hasNumberOfClusters(nOpt.get()) ? "[IGNORED]" : "";
                    c.saveAsJson(generationTargetPath +"emam/", "clusteringResults.json");
                    System.out.println(prefix + "Score was " + c.getScore() + " for " + c.getParameters().toString());
                }

                Optional<ClusteringResult> clusteringOpt;
                if(nOpt.isPresent() && clusteringParameters.getChooseBy().equals(ResultChoosingStrategy.bestWithFittingN)){
                    clusteringOpt = clusteringResults.getBestResultWithFittingN(nOpt.get());
                }else{
                    clusteringOpt = clusteringResults.getBestResultOverall();
                }

                if(clusteringOpt.isPresent()){
                    ClusteringResult clusteringResult = clusteringOpt.get();
                    System.out.println("Best score was " + clusteringResult.getScore() + " for " + clusteringResult.getParameters().toString());
                    AutomaticClusteringHelper.annotateComponentWithRosTagsForClusters(componentInstanceSymbol, clusteringResult.getClustering());
                }
            }
        }

        fixComponentInstance(componentInstanceSymbol);
        return componentInstanceSymbol;
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

    private void fixComponentInstance(EMAComponentInstanceSymbol componentInstanceSymbol) {
        RosHelper.fixRosConnectionSymbols(componentInstanceSymbol);
    }

    @Override
    protected File generateCMake(EMAComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");
        StringBuilder content = new StringBuilder();
        content.append("cmake_minimum_required(VERSION 3.5)\n");
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
