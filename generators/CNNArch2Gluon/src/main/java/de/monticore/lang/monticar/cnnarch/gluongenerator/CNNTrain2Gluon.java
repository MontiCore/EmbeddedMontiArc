/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import com.google.common.base.Splitter;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.generator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.generator.annotations.NNArchitecture;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.FunctionParameterChecker;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.cnnarch.generator.training.RlAlgorithm;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.monticore.lang.monticar.generator.FileContent;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.OPTIMIZER_ADAMW;

public class CNNTrain2Gluon extends CNNTrainGenerator {

    private static final String REINFORCEMENT_LEARNING_FRAMEWORK_MODULE = "reinforcement_learning";
    private static final String GAN_LEARNING_FRAMEWORK_MODULE = "gan";

    private final RewardFunctionSourceGenerator rewardFunctionSourceGenerator;
    private String rootProjectModelsDir;

    public static void main(String[] args) {
        ClassLoader loader = CNNTrain2Gluon.class.getClassLoader();
        System.out.println(loader.getResource("de/monticore/lang/monticar/cnnarch/gluongenerator/CNNTrain2Gluon.class"));
    }

    public CNNTrain2Gluon(RewardFunctionSourceGenerator rewardFunctionSourceGenerator) {
        super(new CNNArch2GluonTrainParamSupportChecker());
        this.rewardFunctionSourceGenerator = rewardFunctionSourceGenerator;
    }

    @Override
    public void generate(Path modelsDirPath, String rootModelName) {
        TrainingConfiguration trainingConfiguration = createTrainingConfiguration(modelsDirPath, rootModelName, null);

        TrainingComponentsContainer trainingComponentsContainer = new TrainingComponentsContainer();
        GluonConfigurationData configurationData = new GluonConfigurationData(trainingConfiguration, trainingComponentsContainer, getInstanceName());
        if (configurationData.isReinforcementLearning()) {
            throw new IllegalStateException("Cannot call generate of reinforcement configuration without specifying the trained architecture");
        }
        generateFilesFromConfigurationSymbol(trainingConfiguration, trainingComponentsContainer, null);
    }

    public void generate(Path modelsDirPath, String rootModelName, ArchitectureAdapter trainedArchitecture) {
        generate(modelsDirPath, rootModelName, trainedArchitecture, null);
    }

    public void generate(Path modelsDirPath, String rootModelName, ArchitectureAdapter trainedArchitecture,
                         ArchitectureAdapter supportNetwork) {
        TrainingConfiguration trainingConfiguration = createTrainingConfiguration(modelsDirPath, rootModelName, null);

        TrainingComponentsContainer trainingComponentsContainer = new TrainingComponentsContainer();
        trainingComponentsContainer.setTrainedArchitecture(trainingConfiguration, trainedArchitecture);
        if (supportNetwork != null) {
            if (trainingConfiguration.isReinforcementLearning()) {
                trainingComponentsContainer.setCriticNetwork(supportNetwork);
            } else if (trainingConfiguration.isVaeLearning()) {
                trainingComponentsContainer.setEncoderNetwork(supportNetwork);
            }
        }

        setRootProjectModelsDir(modelsDirPath.toString());
        generateFilesFromConfigurationSymbol(trainingConfiguration, trainingComponentsContainer, null);
    }

    public void generate(Path modelsDirPath, String rootModelName, ArchitectureAdapter trainedArchitecture, ArchitectureAdapter discriminatorNetwork, ArchitectureAdapter qNetwork) {
        TrainingConfiguration trainingConfiguration = createTrainingConfiguration(modelsDirPath, rootModelName, null);

        TrainingComponentsContainer trainingComponentsContainer = new TrainingComponentsContainer();
        trainingComponentsContainer.setTrainedArchitecture(trainingConfiguration, trainedArchitecture);
        trainingComponentsContainer.setDiscriminatorNetwork(discriminatorNetwork);
        if (qNetwork != null) {
            trainingComponentsContainer.setQNetwork(qNetwork);
        }

        setRootProjectModelsDir(modelsDirPath.toString());
        generateFilesFromConfigurationSymbol(trainingConfiguration, trainingComponentsContainer, null);
    }

    @Override
    public List<FileContent> generateStrings(TrainingConfiguration trainingConfiguration,
                                             TrainingComponentsContainer trainingComponentsContainer,
                                             Path outputPath) {
        validateConfiguration(trainingConfiguration, trainingComponentsContainer, outputPath);

        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        GluonConfigurationData configurationData = new GluonConfigurationData(trainingConfiguration,
                trainingComponentsContainer, getInstanceName());

        List<GluonConfigurationData> configDataList = Lists.newArrayList(configurationData);
        Map<String, Object> ftlContext = Maps.newHashMap();
        ftlContext.put("configurations", configDataList);

        List<FileContent> fileContents = new ArrayList<>();
        //Context Information and Optimizer for local adaption during prediction for replay memory layer (the second only applicable for supervised learning)
        String cnnTrainLAOptimizerTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNLAOptimizer.ftl");
        fileContents.add(new FileContent(cnnTrainLAOptimizerTemplateContent, "CNNLAOptimizer_" + getInstanceName() + ".h"));

        // AdamW optimizer if used for training
        if(trainingConfiguration.hasOptimizer()) {
            String optimizerName = trainingConfiguration.getOptimizerName().get();

            String criticOptimizerName = "";
            if (trainingConfiguration.hasCriticOptimizer()) {
                criticOptimizerName = trainingConfiguration.getCriticOptimizerName().get();
            }

            if (optimizerName.equals(OPTIMIZER_ADAMW) || criticOptimizerName.equals(OPTIMIZER_ADAMW)) {
                String adamWContent = templateConfiguration.processTemplate(ftlContext, "Optimizer/AdamW.ftl");
                fileContents.add(new FileContent(adamWContent, "AdamW.py"));
            }

            if (trainingConfiguration.getOptimizerName().get().equals(new String("hpo"))) {
                String cnnHPOTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNHyperparameterOptimization.ftl");
                fileContents.add(new FileContent(cnnHPOTemplateContent, "CNNHyperparameterOptimization_" + getInstanceName() + ".py"));
            }
        }

        if (trainingConfiguration.isSupervisedLearning()) {
            String cnnTrainTrainerTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
            fileContents.add(new FileContent(cnnTrainTrainerTemplateContent, "CNNTrainer_" + getInstanceName() + ".py")); 

        } else if (trainingConfiguration.isGanLearning()) {

            final String trainerName = "CNNTrainer_" + getInstanceName();
            Optional<ArchitectureAdapter> discriminatorNetworkOpt = trainingComponentsContainer.getDiscriminatorNetwork();
            if (!discriminatorNetworkOpt.isPresent()) {
                Log.error("No architecture model for discriminator available but is required for chosen " +
                        "GAN");
            }

            ArchitectureAdapter genericArchitectureSymbol = discriminatorNetworkOpt.get();
            ArchitectureSymbol disArchitectureSymbol = genericArchitectureSymbol.getArchitectureSymbol();

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

            Optional<ArchitectureAdapter> qNetworkOpt = trainingComponentsContainer.getQNetwork();
            if (qNetworkOpt.isPresent()) {
                ArchitectureAdapter genericQArchitectureSymbol = qNetworkOpt.get();
                ArchitectureSymbol qArchitectureSymbol = genericQArchitectureSymbol.getArchitectureSymbol();

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
            ftlContext.put("learningMethod", "gan");
            ftlContext.put("trainerName", trainerName);  //GANs and VAEs share the same Trainer.ftl

            final String initContent = "";
            fileContents.add(new FileContent(initContent, GAN_LEARNING_FRAMEWORK_MODULE + "/__init__.py"));

            final String ganTrainerContent = templateConfiguration.processTemplate(ftlContext, "generative_model/Trainer.ftl");
            fileContents.add(new FileContent(ganTrainerContent, trainerName + ".py"));

        } else if (trainingConfiguration.isVaeLearning()) {

            final String trainerName = "CNNTrainer_" + getInstanceName();
            Optional<ArchitectureAdapter> encoderNetworkOpt = trainingComponentsContainer.getEncoderNetwork();
            if (!encoderNetworkOpt.isPresent()) {
                Log.error("No architecture model for encoder available but is required for chosen " +
                        "VAE");
            }

            ArchitectureAdapter genericArchitectureSymbol = encoderNetworkOpt.get();
            ArchitectureSymbol encoderArchitectureSymbol = genericArchitectureSymbol.getArchitectureSymbol();

            CNNArch2Gluon gluonGenerator = new CNNArch2Gluon();
            gluonGenerator.setGenerationTargetPath(
                    Paths.get(getGenerationTargetPath(), "").toString());

            List<FileContent> encoderArchitectureFileContents
                    = gluonGenerator.generateStringsAllowMultipleIO(encoderArchitectureSymbol, false);

            final String encoderCreatorName = encoderArchitectureFileContents.get(0).getFileName();
            final String encoderInstanceName = encoderCreatorName.substring(
                    encoderCreatorName.indexOf('_') + 1, encoderCreatorName.lastIndexOf(".py"));

            fileContents.addAll(encoderArchitectureFileContents.stream()
                    .map(k -> new FileContent(k.getFileContent(), k.getFileName()))
                    .collect(Collectors.toList()));

            ftlContext.put("encoderInstanceName", encoderInstanceName);
            ftlContext.put("trainerName", trainerName);
            ftlContext.put("learningMethod", "vae"); //GANs and VAEs share the same Trainer.ftl

            String encoderLAOptimizerTemplateContent = templateConfiguration.processTemplate(ftlContext, "generative_model/vae/CNNLAOptimizer.ftl");
            fileContents.add(new FileContent(encoderLAOptimizerTemplateContent, "CNNLAOptimizer_" + encoderInstanceName + ".h"));
            //final String initContent = "";
            //fileContents.add(new FileContent(initContent, "__init__.py"));

            final String vaeTrainerContent = templateConfiguration.processTemplate(ftlContext, "generative_model/Trainer.ftl");
            fileContents.add(new FileContent(vaeTrainerContent, trainerName + ".py"));

        } else if (trainingConfiguration.isReinforcementLearning()) {
            final String trainerName = "CNNTrainer_" + getInstanceName();
            final Optional<RlAlgorithm> rlAlgorithmOpt = trainingConfiguration.getRlAlgorithm();
//            if (!rlAlgorithmOpt.isPresent()) {
//                throw new GenerationAbortedException("Reinforcement learning algorithm must be defined in a reinforcement learning scenario");
//            }

            if (rlAlgorithmOpt.isPresent() && (rlAlgorithmOpt.get().equals(RlAlgorithm.DDPG) || rlAlgorithmOpt.get().equals(RlAlgorithm.TD3))) {
                Optional<ArchitectureAdapter> criticNetworkOpt = trainingComponentsContainer.getCriticNetwork();
                if (!criticNetworkOpt.isPresent()) {
                    Log.error("No architecture model for critic available but is required for chosen " +
                            "actor-critic algorithm");
                }
                ArchitectureAdapter genericArchitectureSymbol = criticNetworkOpt.get();
                ArchitectureSymbol architectureSymbol = genericArchitectureSymbol.getArchitectureSymbol();

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
            if (trainingConfiguration.hasRewardFunction()) {
                Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
                if (trainedArchitectureOpt.isPresent()) {
                    Optional<String> rewardFunctionName = trainingConfiguration.getRewardFunctionName();
                    generateRewardFunction(trainedArchitectureOpt.get(), rewardFunctionName.get(), trainingComponentsContainer, Paths.get(rootProjectModelsDir));
                } else {
                    Log.error("No architecture model for the trained neural network but is required for " +
                            "reinforcement learning configuration.");
                }
            }

            ftlContext.put("trainerName", trainerName);
            List<FileContent> rlFrameworkContentMap = constructReinforcementLearningFramework(templateConfiguration, ftlContext);
            fileContents.addAll(rlFrameworkContentMap);

            final String reinforcementTrainerContent = templateConfiguration.processTemplate(ftlContext, "reinforcement/Trainer.ftl");
            fileContents.add(new FileContent(reinforcementTrainerContent, trainerName + ".py"));

            final String startTrainerScriptContent = templateConfiguration.processTemplate(ftlContext, "reinforcement/StartTrainer.ftl");
            fileContents.add(new FileContent(startTrainerScriptContent, "start_training.sh"));
        }

        return fileContents;
    }

    public Optional<String> getRootProjectModelsDir() {
        return Optional.ofNullable(rootProjectModelsDir);
    }

    public void setRootProjectModelsDir(String rootProjectModelsDir) {
        this.rootProjectModelsDir = rootProjectModelsDir;
    }

    private void generateRewardFunction(NNArchitecture trainedArchitecture, String rewardFunctionName,
                                        TrainingComponentsContainer trainingComponentsContainer, Path modelsDirPath) {

        GeneratorPythonWrapperStandaloneApi pythonWrapperApi = new GeneratorPythonWrapperStandaloneApi();
        String rewardFunctionOutputPath = Paths.get(this.getGenerationTargetPath(), "reward").toString();

        if (!getRootProjectModelsDir().isPresent()) {
            setRootProjectModelsDir(modelsDirPath.toString());
        }

        final TaggingResolver taggingResolver
                = rewardFunctionSourceGenerator.createTaggingResolver(getRootProjectModelsDir().get());
        final EMAComponentInstanceSymbol emaSymbol
                = rewardFunctionSourceGenerator.resolveSymbol(taggingResolver, rewardFunctionName);
        rewardFunctionSourceGenerator.generate(emaSymbol, taggingResolver, rewardFunctionOutputPath);
        List<String> rewardFunctionNameComponents = Splitter.on(".").splitToList(rewardFunctionName);
        fixArmadilloEmamGenerationOfFile(Paths.get(rewardFunctionOutputPath, String.join("_", rewardFunctionNameComponents) + ".h"));

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
        trainingComponentsContainer.setRewardFunctionParameter(functionParameter);
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

    private List<FileContent> constructReinforcementLearningFramework(final TemplateConfiguration templateConfiguration,
                                                                              final Map<String, Object> ftlContext) {

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

        final String initContent = "import os, sys; sys.path.append(os.path.dirname(os.path.abspath(__file__)))";
        fileContents.add(new FileContent(initContent, REINFORCEMENT_LEARNING_FRAMEWORK_MODULE + "/__init__.py"));

        return fileContents;
    }
}