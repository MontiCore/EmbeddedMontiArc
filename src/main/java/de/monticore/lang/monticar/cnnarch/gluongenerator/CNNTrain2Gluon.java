package de.monticore.lang.monticar.cnnarch.gluongenerator;

import com.google.common.collect.Maps;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.critic.CriticNetworkGenerationPair;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.critic.CriticNetworkGenerator;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.FunctionParameterChecker;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.ConfigurationData;

import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNTrain2MxNet;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.TemplateConfiguration;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.LearningMethod;
import de.monticore.lang.monticar.cnntrain._symboltable.RLAlgorithm;
import de.monticore.lang.monticar.cnntrain._symboltable.RewardFunctionSymbol;
import de.monticore.lang.monticar.cnntrain.annotations.TrainedArchitecture;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.pythonwrapper.GeneratorPythonWrapperStandaloneApi;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

public class CNNTrain2Gluon extends CNNTrain2MxNet {
    private static final String REINFORCEMENT_LEARNING_FRAMEWORK_MODULE = "reinforcement_learning";

    private final RewardFunctionSourceGenerator rewardFunctionSourceGenerator;
    private String rootProjectModelsDir;

    public Optional<String> getRootProjectModelsDir() {
        return Optional.ofNullable(rootProjectModelsDir);
    }

    public void setRootProjectModelsDir(String rootProjectModelsDir) {
        this.rootProjectModelsDir = rootProjectModelsDir;
    }

    public CNNTrain2Gluon(RewardFunctionSourceGenerator rewardFunctionSourceGenerator) {
        super();
        this.rewardFunctionSourceGenerator = rewardFunctionSourceGenerator;
    }

    @Override
    public ConfigurationSymbol getConfigurationSymbol(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configurationSymbol = super.getConfigurationSymbol(modelsDirPath, rootModelName);

        // Generate Reward function if necessary
        if (configurationSymbol.getLearningMethod().equals(LearningMethod.REINFORCEMENT)
                && configurationSymbol.getRlRewardFunction().isPresent()) {
            generateRewardFunction(configurationSymbol.getRlRewardFunction().get(), modelsDirPath);
        }

        return configurationSymbol;
    }

    @Override
    public void generate(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configuration = this.getConfigurationSymbol(modelsDirPath, rootModelName);

        if (configuration.getLearningMethod().equals(LearningMethod.REINFORCEMENT)) {
            throw new IllegalStateException("Cannot call generate of reinforcement configuration without specifying " +
             "the trained architecture");
        }

        generateFilesFromConfigurationSymbol(configuration);
    }

    private void generateFilesFromConfigurationSymbol(ConfigurationSymbol configuration) {
        Map<String, String> fileContents = this.generateStrings(configuration);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(this.getGenerationTargetPath());

        try {
            Iterator var6 = fileContents.keySet().iterator();

            while(var6.hasNext()) {
                String fileName = (String)var6.next();
                genCPP.generateFile(new FileContent((String)fileContents.get(fileName), fileName));
            }
        } catch (IOException var8) {
            Log.error("CNNTrainer file could not be generated" + var8.getMessage());
        }
    }

    public void generate(Path modelsDirPath, String rootModelName, TrainedArchitecture trainedArchitecture) {
        ConfigurationSymbol configurationSymbol = this.getConfigurationSymbol(modelsDirPath, rootModelName);
        configurationSymbol.setTrainedArchitecture(trainedArchitecture);
        this.setRootProjectModelsDir(modelsDirPath.toString());
        generateFilesFromConfigurationSymbol(configurationSymbol);
    }

    @Override
    public Map<String, String> generateStrings(ConfigurationSymbol configuration) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        ReinforcementConfigurationData configData = new ReinforcementConfigurationData(configuration, getInstanceName());
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);

        Map<String, Object> ftlContext = Maps.newHashMap();
        ftlContext.put("configurations", configDataList);

        Map<String, String> fileContentMap = new HashMap<>();

        if (configData.isSupervisedLearning()) {
            String cnnTrainTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
            fileContentMap.put("CNNTrainer_" + getInstanceName() + ".py", cnnTrainTemplateContent);
        } else if (configData.isReinforcementLearning()) {
            final String trainerName = "CNNTrainer_" + getInstanceName();
            final RLAlgorithm rlAlgorithm = configData.getRlAlgorithm();

            if (rlAlgorithm.equals(RLAlgorithm.DDPG)) {
                CriticNetworkGenerator criticNetworkGenerator = new CriticNetworkGenerator();
                criticNetworkGenerator.setGenerationTargetPath(
                    Paths.get(getGenerationTargetPath(), REINFORCEMENT_LEARNING_FRAMEWORK_MODULE).toString());
                if (getRootProjectModelsDir().isPresent()) {
                    criticNetworkGenerator.setRootModelsDir(getRootProjectModelsDir().get());
                } else {
                    Log.error("No root model dir set");
                }

                CriticNetworkGenerationPair criticNetworkResult
                    = criticNetworkGenerator.generateCriticNetworkContent(templateConfiguration, configuration);

                fileContentMap.putAll(criticNetworkResult.getFileContent().entrySet().stream().collect(Collectors.toMap(
                            k -> REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/" + k.getKey(),
                            Map.Entry::getValue))
                );
                ftlContext.put("criticInstanceName", criticNetworkResult.getCriticNetworkName());
            }

            ftlContext.put("trainerName", trainerName);
            Map<String, String> rlFrameworkContentMap = constructReinforcementLearningFramework(templateConfiguration, ftlContext, rlAlgorithm);
            fileContentMap.putAll(rlFrameworkContentMap);

            final String reinforcementTrainerContent = templateConfiguration.processTemplate(ftlContext, "reinforcement/Trainer.ftl");
            fileContentMap.put(trainerName + ".py", reinforcementTrainerContent);

            final String startTrainerScriptContent = templateConfiguration.processTemplate(ftlContext, "reinforcement/StartTrainer.ftl");
            fileContentMap.put("start_training.sh", startTrainerScriptContent);
        }
        return fileContentMap;
    }

    private void generateRewardFunction(RewardFunctionSymbol rewardFunctionSymbol, Path modelsDirPath) {
        GeneratorPythonWrapperStandaloneApi pythonWrapperApi = new GeneratorPythonWrapperStandaloneApi();

        List<String> fullNameOfComponent = rewardFunctionSymbol.getRewardFunctionComponentName();

        String rewardFunctionRootModel = String.join(".", fullNameOfComponent);
        String rewardFunctionOutputPath = Paths.get(this.getGenerationTargetPath(), "reward").toString();

        if (!getRootProjectModelsDir().isPresent()) {
            setRootProjectModelsDir(modelsDirPath.toString());
        }

        rewardFunctionSourceGenerator.generate(getRootProjectModelsDir().get(),
                rewardFunctionRootModel, rewardFunctionOutputPath);
        fixArmadilloEmamGenerationOfFile(Paths.get(rewardFunctionOutputPath, String.join("_", fullNameOfComponent) + ".h"));

        String pythonWrapperOutputPath = Paths.get(rewardFunctionOutputPath, "pylib").toString();

        Log.info("Generating reward function python wrapper...", "CNNTrain2Gluon");
        ComponentPortInformation componentPortInformation;
        if (pythonWrapperApi.checkIfPythonModuleBuildAvailable()) {
            final String rewardModuleOutput
                    = Paths.get(getGenerationTargetPath(), REINFORCEMENT_LEARNING_FRAMEWORK_MODULE).toString();
            componentPortInformation = pythonWrapperApi.generateAndTryBuilding(getRootProjectModelsDir().get(),
                    rewardFunctionRootModel, pythonWrapperOutputPath, rewardModuleOutput);
        } else {
            Log.warn("Cannot build wrapper automatically: OS not supported. Please build manually before starting training.");
            componentPortInformation = pythonWrapperApi.generate(getRootProjectModelsDir().get(), rewardFunctionRootModel,
                    pythonWrapperOutputPath);
        }
        RewardFunctionParameterAdapter functionParameter = new RewardFunctionParameterAdapter(componentPortInformation);
        new FunctionParameterChecker().check(functionParameter);
        rewardFunctionSymbol.setRewardFunctionParameter(functionParameter);
    }

    private void fixArmadilloEmamGenerationOfFile(Path pathToBrokenFile){
        final File brokenFile = pathToBrokenFile.toFile();
        if (brokenFile.exists()) {
            try {
                Charset charset = StandardCharsets.UTF_8;
                String fileContent = new String(Files.readAllBytes(pathToBrokenFile), charset);
                fileContent = fileContent.replace("armadillo.h", "armadillo");
                Files.write(pathToBrokenFile, fileContent.getBytes());
            } catch (IOException e) {
                Log.warn("Cannot fix wrong armadillo library in " + pathToBrokenFile.toString());
            }
        }
    }

    private Map<String, String> constructReinforcementLearningFramework(
            final TemplateConfiguration templateConfiguration,
            final Map<String, Object> ftlContext,
            RLAlgorithm rlAlgorithm) {
        Map<String, String> fileContentMap = Maps.newHashMap();
        ftlContext.put("rlFrameworkModule", REINFORCEMENT_LEARNING_FRAMEWORK_MODULE);

        final String loggerContent = templateConfiguration.processTemplate(ftlContext,
                "reinforcement/util/Logger.ftl");
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/cnnarch_logger.py", loggerContent);

        final String reinforcementAgentContent = templateConfiguration.processTemplate(ftlContext,
                "reinforcement/agent/Agent.ftl");
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/agent.py", reinforcementAgentContent);

        final String reinforcementStrategyContent = templateConfiguration.processTemplate(
            ftlContext, "reinforcement/agent/Strategy.ftl");
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/strategy.py", reinforcementStrategyContent);

        final String replayMemoryContent = templateConfiguration.processTemplate(
            ftlContext, "reinforcement/agent/ReplayMemory.ftl");
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/replay_memory.py", replayMemoryContent);

        final String environmentContent = templateConfiguration.processTemplate(
            ftlContext, "reinforcement/environment/Environment.ftl");
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/environment.py", environmentContent);

        final String utilContent = templateConfiguration.processTemplate(
            ftlContext, "reinforcement/util/Util.ftl");
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/util.py", utilContent);

        final String initContent = "";
        fileContentMap.put(REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/__init__.py", initContent);

        return fileContentMap;
    }
}