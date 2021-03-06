/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainNode;
import de.monticore.lang.monticar.cnntrain._ast.ASTOptimizerEntry;
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

public abstract class CNNTrainGenerator {

    protected TrainParamSupportChecker trainParamSupportChecker;

    private String generationTargetPath;
    private String instanceName;

    protected CNNTrainGenerator() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    private void supportCheck(ConfigurationSymbol configuration){
        checkEntryParams(configuration);
        checkOptimizerParams(configuration);
    }

    private void checkEntryParams(ConfigurationSymbol configuration){
        Iterator it = configuration.getEntryMap().keySet().iterator();
        while (it.hasNext()) {
            String key = it.next().toString();
            ASTCNNTrainNode astTrainEntryNode = (ASTCNNTrainNode) configuration.getEntryMap().get(key).getAstNode().get();
            astTrainEntryNode.accept(trainParamSupportChecker);
        }
        it = configuration.getEntryMap().keySet().iterator();
        while (it.hasNext()) {
            String key = it.next().toString();
            if (trainParamSupportChecker.getUnsupportedElemList().contains(key)) {
                it.remove();
            }
        }
    }

    private void checkOptimizerParams(ConfigurationSymbol configuration){
        if (configuration.getOptimizer() != null) {
            ASTOptimizerEntry astOptimizer = (ASTOptimizerEntry) configuration.getOptimizer().getAstNode().get();
            astOptimizer.accept(trainParamSupportChecker);
            if (trainParamSupportChecker.getUnsupportedElemList().contains(trainParamSupportChecker.unsupportedOptFlag)) {
                configuration.setOptimizer(null);
            }else {
                Iterator it = configuration.getOptimizer().getOptimizerParamMap().keySet().iterator();
                while (it.hasNext()) {
                    String key = it.next().toString();
                    if (trainParamSupportChecker.getUnsupportedElemList().contains(key)) {
                        it.remove();
                    }
                }
            }
        }
    }

    private static void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
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
            quitGeneration();
        }
        setInstanceName(compilationUnit.get().getFullName());
        CNNTrainCocos.checkAll(compilationUnit.get());
        supportCheck(compilationUnit.get().getConfiguration());
		
        return compilationUnit.get().getConfiguration();
    }

    public abstract void generate(Path modelsDirPath, String rootModelNames);

    //check cocos with CNNTrainCocos.checkAll(configuration) before calling this method.
    public abstract List<FileContent> generateStrings(ConfigurationSymbol configuration);
}
