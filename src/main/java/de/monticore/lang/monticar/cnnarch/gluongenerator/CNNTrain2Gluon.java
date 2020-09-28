/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import com.google.common.collect.Maps;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.gluongenerator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.FunctionParameterChecker;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;

import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnntrain._symboltable.*;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.pythonwrapper.GeneratorPythonWrapperStandaloneApi;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
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

public class CNNTrain2Gluon extends CNNTrainGenerator {
    private static final String REINFORCEMENT_LEARNING_FRAMEWORK_MODULE = "reinforcement_learning";
    private static final String GAN_LEARNING_FRAMEWORK_MODULE = "gan";

    private final RewardFunctionSourceGenerator rewardFunctionSourceGenerator;
    private String rootProjectModelsDir;

    public Optional<String> getRootProjectModelsDir() {
        return Optional.ofNullable(rootProjectModelsDir);
    }

    public void setRootProjectModelsDir(String rootProjectModelsDir) {
        this.rootProjectModelsDir = rootProjectModelsDir;
    }

    public CNNTrain2Gluon(RewardFunctionSourceGenerator rewardFunctionSourceGenerator) {
        trainParamSupportChecker = new CNNArch2GluonTrainParamSupportChecker();

        this.rewardFunctionSourceGenerator = rewardFunctionSourceGenerator;
    }

    @Override
    public ConfigurationSymbol getConfigurationSymbol(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configurationSymbol = super.getConfigurationSymbol(modelsDirPath, rootModelName);
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
        List<FileContent> fileContents = this.generateStrings(configuration);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(this.getGenerationTargetPath());

        try {
            for (FileContent fileContent : fileContents) {
                genCPP.generateFile(fileContent);
            }
        } catch (IOException var8) {
            Log.error("CNNTrainer file could not be generated" + var8.getMessage());
        }
    }

    public void generate(Path modelsDirPath,
                         String rootModelName,
                         NNArchitectureSymbol trainedArchitecture,
                         NNArchitectureSymbol criticNetwork) {
        ConfigurationSymbol configurationSymbol = this.getConfigurationSymbol(modelsDirPath, rootModelName);
        configurationSymbol.setTrainedArchitecture(trainedArchitecture);
        configurationSymbol.setCriticNetwork(criticNetwork);
        this.setRootProjectModelsDir(modelsDirPath.toString());
        generateFilesFromConfigurationSymbol(configurationSymbol);
    }

    public void generate(Path modelsDirPath,
                         String rootModelName,
                         NNArchitectureSymbol trainedArchitecture,
                         NNArchitectureSymbol discriminatorNetwork,
                         NNArchitectureSymbol qNetwork) {
        ConfigurationSymbol configurationSymbol = this.getConfigurationSymbol(modelsDirPath, rootModelName);
        configurationSymbol.setTrainedArchitecture(trainedArchitecture);
        configurationSymbol.setDiscriminatorNetwork(discriminatorNetwork);
        configurationSymbol.setQNetwork(qNetwork);
        this.setRootProjectModelsDir(modelsDirPath.toString());
        generateFilesFromConfigurationSymbol(configurationSymbol);
    }

    public void generate(Path modelsDirPath, String rootModelName, NNArchitectureSymbol trainedArchitecture) {
        generate(modelsDirPath, rootModelName, trainedArchitecture, null);
    }

    @Override
    public List<FileContent> generateStrings(ConfigurationSymbol configuration) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        GluonConfigurationData configData = new GluonConfigurationData(configuration, getInstanceName());
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);

        Map<String, Object> ftlContext = Maps.newHashMap();
        ftlContext.put("configurations", configDataList);

        List<FileContent> fileContents = new ArrayList<>();

        //Context Information and Optimizer for local adaption during prediction for replay memory layer (the second only applicaple for supervised learning)
        String cnnTrainLAOptimizerTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNLAOptimizer.ftl");
        fileContents.add(new FileContent(cnnTrainLAOptimizerTemplateContent, "CNNLAOptimizer_" + getInstanceName() + ".h"));

        //AdamW optimizer if used for training
        if(configuration.getOptimizer() != null) {
            String optimizerName = configuration.getOptimizer().getName();
            Optional<OptimizerSymbol> criticOptimizer = configuration.getCriticOptimizer();
            String criticOptimizerName = "";
            if (criticOptimizer.isPresent()) {
                criticOptimizerName = criticOptimizer.get().getName();
            }
            if (optimizerName.equals("adamw") || criticOptimizerName.equals("adamw")) {
                String adamWContent = templateConfiguration.processTemplate(ftlContext, "Optimizer/AdamW.ftl");
                fileContents.add(new FileContent(adamWContent, "AdamW.py"));
            }
        }

        if (configData.isSupervisedLearning()) {
            String cnnTrainTrainerTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
            fileContents.add(new FileContent(cnnTrainTrainerTemplateContent, "CNNTrainer_" + getInstanceName() + ".py"));
        } else if (configData.isGan()) {
            final String trainerName = "CNNTrainer_" + getInstanceName();
            if (!configuration.getDiscriminatorNetwork().isPresent()) {
                Log.error("No architecture model for discriminator available but is required for chosen " +
                        "GAN");
            }

            NNArchitectureSymbol genericDisArchitectureSymbol = configuration.getDiscriminatorNetwork().get();
            ArchitectureSymbol disArchitectureSymbol
                    = ((ArchitectureAdapter) genericDisArchitectureSymbol).getArchitectureSymbol();

            CNNArch2Gluon gluonGenerator = new CNNArch2Gluon();
            gluonGenerator.setGenerationTargetPath(
                    Paths.get(getGenerationTargetPath(), GAN_LEARNING_FRAMEWORK_MODULE).toString());

            List<FileContent> disArchitectureFileContents
                    = gluonGenerator.generateStringsAllowMultipleIO(disArchitectureSymbol, true);

            final String disCreatorName = disArchitectureFileContents.get(0).getFileName();
            final String discriminatorInstanceName = disCreatorName.substring(
                    disCreatorName.indexOf('_') + 1, disCreatorName.lastIndexOf(".py"));

            fileContents.addAll(disArchitectureFileContents.stream()
                    .map(k -> new FileContent(k.getFileContent(), GAN_LEARNING_FRAMEWORK_MODULE + "/" + k.getFileName()))
                    .collect(Collectors.toList()));

            if (configuration.hasQNetwork()) {
                NNArchitectureSymbol genericQArchitectureSymbol = configuration.getQNetwork().get();
                ArchitectureSymbol qArchitectureSymbol
                        = ((ArchitectureAdapter) genericQArchitectureSymbol).getArchitectureSymbol();

                List<FileContent> qArchitectureFileContents
                        = gluonGenerator.generateStringsAllowMultipleIO(qArchitectureSymbol, true);

                final String qCreatorName = qArchitectureFileContents.get(0).getFileName();
                final String qNetworkInstanceName = qCreatorName.substring(
                        qCreatorName.indexOf('_') + 1, qCreatorName.lastIndexOf(".py"));

                fileContents.addAll(qArchitectureFileContents.stream()
                        .map(k -> new FileContent(k.getFileContent(), GAN_LEARNING_FRAMEWORK_MODULE + "/" + k.getFileName()))
                        .collect(Collectors.toList()));

                ftlContext.put("qNetworkInstanceName", qNetworkInstanceName);
            }

            ftlContext.put("ganFrameworkModule", GAN_LEARNING_FRAMEWORK_MODULE);
            ftlContext.put("discriminatorInstanceName", discriminatorInstanceName);
            ftlContext.put("trainerName", trainerName);

            final String initContent = "";
            fileContents.add(new FileContent(initContent, GAN_LEARNING_FRAMEWORK_MODULE + "/__init__.py"));

            final String ganTrainerContent = templateConfiguration.processTemplate(ftlContext, "gan/Trainer.ftl");
            fileContents.add(new FileContent(ganTrainerContent, trainerName + ".py"));
        } else if (configData.isReinforcementLearning()) {
            final String trainerName = "CNNTrainer_" + getInstanceName();
            final RLAlgorithm rlAlgorithm = configData.getRlAlgorithm();

            if (rlAlgorithm.equals(RLAlgorithm.DDPG)
                    || rlAlgorithm.equals(RLAlgorithm.TD3)) {

                if (!configuration.getCriticNetwork().isPresent()) {
                    Log.error("No architecture model for critic available but is required for chosen " +
                            "actor-critic algorithm");
                }
                NNArchitectureSymbol genericArchitectureSymbol = configuration.getCriticNetwork().get();
                ArchitectureSymbol architectureSymbol
                        = ((ArchitectureAdapter) genericArchitectureSymbol).getArchitectureSymbol();

                CNNArch2Gluon gluonGenerator = new CNNArch2Gluon();
                gluonGenerator.setGenerationTargetPath(
                        Paths.get(getGenerationTargetPath(), REINFORCEMENT_LEARNING_FRAMEWORK_MODULE).toString());
                List<FileContent> architectureFileContents
                        = gluonGenerator.generateStringsAllowMultipleIO(architectureSymbol, true);


                final String creatorName = architectureFileContents.get(0).getFileName();
                final String criticInstanceName = creatorName.substring(
                        creatorName.indexOf('_') + 1, creatorName.lastIndexOf(".py"));

                fileContents.addAll(architectureFileContents.stream()
                        .map(k -> new FileContent(k.getFileContent(), REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/" + k.getFileName()))
                        .collect(Collectors.toList()));

                ftlContext.put("criticInstanceName", criticInstanceName);
            }

            // Generate Reward function if necessary
            if (configuration.getRlRewardFunction().isPresent()) {
                if (configuration.getTrainedArchitecture().isPresent()) {
                    generateRewardFunction(configuration.getTrainedArchitecture().get(),
                            configuration.getRlRewardFunction().get(), Paths.get(rootProjectModelsDir));
                } else {
                    Log.error("No architecture model for the trained neural network but is required for " +
                            "reinforcement learning configuration.");
                }

            }

            ftlContext.put("trainerName", trainerName);
            List<FileContent> rlFrameworkContentMap = constructReinforcementLearningFramework(templateConfiguration, ftlContext, rlAlgorithm);
            fileContents.addAll(rlFrameworkContentMap);

            final String reinforcementTrainerContent = templateConfiguration.processTemplate(ftlContext, "reinforcement/Trainer.ftl");
            fileContents.add(new FileContent(reinforcementTrainerContent, trainerName + ".py"));

            final String startTrainerScriptContent = templateConfiguration.processTemplate(ftlContext, "reinforcement/StartTrainer.ftl");
            fileContents.add(new FileContent(startTrainerScriptContent, "start_training.sh"));
        }
        return fileContents;
    }

    private void generateRewardFunction(NNArchitectureSymbol trainedArchitecture,
                                        RewardFunctionSymbol rewardFunctionSymbol, Path modelsDirPath) {
        GeneratorPythonWrapperStandaloneApi pythonWrapperApi = new GeneratorPythonWrapperStandaloneApi();

        List<String> fullNameOfComponent = rewardFunctionSymbol.getRewardFunctionComponentName();

        String rewardFunctionRootModel = String.join(".", fullNameOfComponent);
        String rewardFunctionOutputPath = Paths.get(this.getGenerationTargetPath(), "reward").toString();

        if (!getRootProjectModelsDir().isPresent()) {
            setRootProjectModelsDir(modelsDirPath.toString());
        }

        final TaggingResolver taggingResolver
                = rewardFunctionSourceGenerator.createTaggingResolver(getRootProjectModelsDir().get());
        final EMAComponentInstanceSymbol emaSymbol
                = rewardFunctionSourceGenerator.resolveSymbol(taggingResolver, rewardFunctionRootModel);
        rewardFunctionSourceGenerator.generate(emaSymbol, taggingResolver, rewardFunctionOutputPath);
        fixArmadilloEmamGenerationOfFile(Paths.get(rewardFunctionOutputPath, String.join("_", fullNameOfComponent) + ".h"));

        String pythonWrapperOutputPath = Paths.get(rewardFunctionOutputPath, "pylib").toString();

        Log.info("Generating reward function python wrapper...", "CNNTrain2Gluon");
        ComponentPortInformation componentPortInformation;
        if (pythonWrapperApi.checkIfPythonModuleBuildAvailable()) {
            final String rewardModuleOutput
                    = Paths.get(getGenerationTargetPath(), REINFORCEMENT_LEARNING_FRAMEWORK_MODULE).toString();
            componentPortInformation = pythonWrapperApi.generateAndTryBuilding(emaSymbol,
                    pythonWrapperOutputPath, rewardModuleOutput);
        } else {
            Log.warn("Cannot build wrapper automatically: OS not supported. Please build manually before starting training.");
            componentPortInformation = pythonWrapperApi.generate(emaSymbol, pythonWrapperOutputPath);
        }
        RewardFunctionParameterAdapter functionParameter = new RewardFunctionParameterAdapter(componentPortInformation);
        new FunctionParameterChecker().check(functionParameter, trainedArchitecture);
        rewardFunctionSymbol.setRewardFunctionParameter(functionParameter);
    }

    private void fixArmadilloEmamGenerationOfFile(Path pathToBrokenFile) {
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

    private List<FileContent> constructReinforcementLearningFramework(
            final TemplateConfiguration templateConfiguration,
            final Map<String, Object> ftlContext,
            RLAlgorithm rlAlgorithm) {
        List<FileContent> fileContents = new ArrayList<>();
        ftlContext.put("rlFrameworkModule", REINFORCEMENT_LEARNING_FRAMEWORK_MODULE);

        final String loggerContent = templateConfiguration.processTemplate(ftlContext,
                "reinforcement/util/Logger.ftl");
        fileContents.add(new FileContent(loggerContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/cnnarch_logger.py"));

        final String reinforcementAgentContent = templateConfiguration.processTemplate(ftlContext,
                "reinforcement/agent/Agent.ftl");
        fileContents.add(new FileContent(reinforcementAgentContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/agent.py"));

        final String reinforcementStrategyContent = templateConfiguration.processTemplate(
                ftlContext, "reinforcement/agent/Strategy.ftl");
        fileContents.add(new FileContent(reinforcementStrategyContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/strategy.py"));

        final String replayMemoryContent = templateConfiguration.processTemplate(
                ftlContext, "reinforcement/agent/ReplayMemory.ftl");
        fileContents.add(new FileContent(replayMemoryContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/replay_memory.py"));

        final String environmentContent = templateConfiguration.processTemplate(
                ftlContext, "reinforcement/environment/Environment.ftl");
        fileContents.add(new FileContent(environmentContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/environment.py"));

        final String utilContent = templateConfiguration.processTemplate(
                ftlContext, "reinforcement/util/Util.ftl");
        fileContents.add(new FileContent(utilContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/util.py"));

        final String initContent = "";
        fileContents.add(new FileContent(initContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/__init__.py"));

        return fileContents;
    }
}

