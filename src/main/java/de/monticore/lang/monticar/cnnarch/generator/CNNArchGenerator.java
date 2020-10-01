/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public abstract class CNNArchGenerator implements Generator<ArchitectureSymbol> {

    private boolean generateCMake = false;

    protected ArchitectureSupportChecker architectureSupportChecker;
    protected LayerSupportChecker layerSupportChecker;

    private String generationTargetPath;
    private String modelsDirPath;

    protected CNNArchGenerator() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    public static void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    @Override
    public boolean isGenerateCMake() {
        return generateCMake;
    }

    @Override
    public void setGenerateCMake(boolean generateCMake) {
        this.generateCMake = generateCMake;
    }

    public String getGenerationTargetPath(){
        if (generationTargetPath.charAt(generationTargetPath.length() - 1) != '/') {
            this.generationTargetPath = generationTargetPath + "/";
        }
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath){
        this.generationTargetPath = generationTargetPath;
    }

    protected String getModelsDirPath() {
        return this.modelsDirPath;
    }

    public void generate(Path modelsDirPath, String rootModelName){
        this.modelsDirPath = modelsDirPath.toString();
        final ModelPath mp = new ModelPath(modelsDirPath);
        GlobalScope scope = new GlobalScope(mp, new CNNArchLanguage());
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(rootModelName));
        generate(tagging, rootModelName);
    }

    // TODO: Rewrite so that CNNArchSymbolCompiler is used in EMADL2CPP instead of this method
    public boolean check(ArchitectureSymbol architecture) {
        return architectureSupportChecker.check(architecture) && layerSupportChecker.check(architecture);
    }

    public void generate(TaggingResolver taggingResolver, String rootModelName){
        CNNArchSymbolCompiler symbolCompiler = new CNNArchSymbolCompiler(architectureSupportChecker, layerSupportChecker);
        ArchitectureSymbol architectureSymbol = symbolCompiler.compileArchitectureSymbol(taggingResolver, rootModelName);

        try{
            String dataConfPath = getModelsDirPath() + "/data_paths.txt";
            DataPathConfigParser dataParserConfig = new DataPathConfigParser(dataConfPath);
            String dataPath = dataParserConfig.getDataPath(rootModelName);
            architectureSymbol.setDataPath(dataPath);
            Path weightsConfPath = Paths.get(getModelsDirPath() + "/weights_paths.txt");
            String weightsPath = null;
            if (weightsConfPath.toFile().exists()) {
                WeightsPathConfigParser weightsParserConfig = new WeightsPathConfigParser(getModelsDirPath() + "/weights_paths.txt");
                weightsPath = weightsParserConfig.getWeightsPath(rootModelName);
            } else {
                Log.info("No weights path definition found in " + weightsConfPath + " found: "
                        + "No pretrained weights will be loaded.", "CNNArchGenerator");
            }
            architectureSymbol.setWeightsPath(weightsPath);
            architectureSymbol.setComponentName(rootModelName);
            generateFiles(taggingResolver, architectureSymbol);
        } catch (IOException e){
            Log.error(e.toString());
        }
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public abstract List<FileContent> generateStrings(TaggingResolver taggingResolver, ArchitectureSymbol architecture);

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public List<File> generateFiles(TaggingResolver taggingResolver, ArchitectureSymbol architecture) throws IOException{
        List<FileContent> fileContents = generateStrings(taggingResolver, architecture);
        return generateFromFilecontentsMap(fileContents);
    }

    public List<File> generateFromFilecontentsMap(List<FileContent> fileContents) throws IOException {
        List<File> res = new LinkedList<>();
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());
        for (FileContent fileContent : fileContents) {
            res.add(genCPP.generateFile(fileContent));
        }
        return res;
    }

    public void generateCMake(String rootModelName){
        List<FileContent> fileContents = generateCMakeContent(rootModelName);
        try {
            generateFromFilecontentsMap(fileContents);
        } catch (IOException e) {
            Log.error("CMake file could not be generated" + e.getMessage());
        }
    }

    public abstract List<FileContent> generateCMakeContent(String rootModelName);


}
