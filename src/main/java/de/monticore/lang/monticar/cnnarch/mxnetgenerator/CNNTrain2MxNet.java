package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnntrain.CNNTrainGenerator;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

public class CNNTrain2MxNet implements CNNTrainGenerator {
    private String generationTargetPath;
    private String instanceName;

    public CNNTrain2MxNet() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    public String getInstanceName() {
        String parsedInstanceName = this.instanceName.replace('.', '_').replace('[', '_').replace(']', '_');
        parsedInstanceName = parsedInstanceName.substring(0, 1).toLowerCase() + parsedInstanceName.substring(1);
        return parsedInstanceName;
    }

    public void setInstanceName(String instanceName) {
        this.instanceName = instanceName;
    }

    public String getGenerationTargetPath() {
        if (generationTargetPath.charAt(generationTargetPath.length() - 1) != '/') {
            this.generationTargetPath = generationTargetPath + "/";
        }
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public ConfigurationSymbol getConfigurationSymbol(Path modelsDirPath, String rootModelName) {
        final ModelPath mp = new ModelPath(modelsDirPath);
        GlobalScope scope = new GlobalScope(mp, new CNNTrainLanguage());
        Optional<CNNTrainCompilationUnitSymbol> compilationUnit = scope.resolve(rootModelName, CNNTrainCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()) {
            Log.error("could not resolve training configuration " + rootModelName);
            System.exit(1);
        }
        setInstanceName(compilationUnit.get().getFullName());
        CNNTrainCocos.checkAll(compilationUnit.get());
        return compilationUnit.get().getConfiguration();
    }

    public void generate(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configuration = getConfigurationSymbol(modelsDirPath, rootModelName);
        Map<String, String> fileContents = generateStrings(configuration);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());
        try {
            for (String fileName : fileContents.keySet()){
                genCPP.generateFile(new FileContent(fileContents.get(fileName), fileName));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Map<String, String> generateStrings(ConfigurationSymbol configuration) {
        ConfigurationData configData = new ConfigurationData(configuration, getInstanceName());
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);
        Map<String, Object> ftlContext = Collections.singletonMap("configurations", configDataList);

        String templateContent = TemplateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        return Collections.singletonMap("CNNTrainer_" + getInstanceName() + ".py", templateContent);
    }

}