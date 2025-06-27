package de.monticore.lang.monticar.cnnarch.generator.training;

import com.google.common.collect.Maps;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch.generator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.generator.preprocessing.PreprocessingComponentParameterAdapter;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.RewardFunctionParameterAdapter;
import schemalang.validation.model.ArchitectureComponent;

import java.util.Map;
import java.util.Optional;

import static de.monticore.lang.monticar.cnnarch.generator.training.MappingUtils.createArchitectureComponent;
import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.*;

public class TrainingComponentsContainer {

    private ArchitectureAdapter trainedArchitecture;
    private ArchitectureAdapter actorNetwork;
    private ArchitectureAdapter criticNetwork;
    private ArchitectureAdapter generatorNetwork;
    private ArchitectureAdapter encoderNetwork;
    private ArchitectureAdapter decoderNetwork;
    private ArchitectureAdapter discriminatorNetwork;
    private ArchitectureAdapter qNetwork;
    private EMAComponentInstanceSymbol rewardFunction;
    private PreprocessingComponentParameterAdapter preprocessingComponentParameter;
    private RewardFunctionParameterAdapter rewardFunctionParameter;
    private Map<String, ArchitectureComponent> architectureComponents = Maps.newHashMap();

    public Optional<ArchitectureAdapter> getActorNetwork() {
        return Optional.ofNullable(actorNetwork);
    }

    public Optional<ArchitectureAdapter> getCriticNetwork() {
        return Optional.ofNullable(criticNetwork);
    }

    public Optional<ArchitectureAdapter> getTrainedArchitecture() {
        return Optional.ofNullable(trainedArchitecture);
    }

    public Optional<ArchitectureAdapter> getGeneratorNetwork() {
        return Optional.ofNullable(generatorNetwork);
    }

    public Optional<ArchitectureAdapter> getDiscriminatorNetwork() {
        return Optional.ofNullable(discriminatorNetwork);
    }

    public Optional<ArchitectureAdapter> getDecoderNetwork() {
        return Optional.ofNullable(decoderNetwork);
    }

    public Optional<ArchitectureAdapter> getEncoderNetwork() {
        return Optional.ofNullable(encoderNetwork);
    }

    public Optional<ArchitectureAdapter> getQNetwork() {
        return Optional.ofNullable(qNetwork);
    }

    public Optional<EMAComponentInstanceSymbol> getRewardFunction() {
        return Optional.ofNullable(rewardFunction);
    }

    public Optional<RewardFunctionParameterAdapter> getRewardFunctionParameter() {
        return Optional.ofNullable(rewardFunctionParameter);
    }

    public void setTrainedArchitecture(TrainingConfiguration trainingConfiguration,
                                        ArchitectureAdapter trainedArchitecture) {
        this.trainedArchitecture = trainedArchitecture;
        if (trainingConfiguration.isReinforcementLearning()) {
            Optional<RlAlgorithm> rlAlgorithmOpt = trainingConfiguration.getRlAlgorithm();
            if (!rlAlgorithmOpt.isPresent()) {
                setQNetwork(trainedArchitecture);
                return;
            }

            RlAlgorithm rlAlgorithm = rlAlgorithmOpt.get();
            if (rlAlgorithm.equals(RlAlgorithm.DQN)) {
                setQNetwork(trainedArchitecture);
            } else if (rlAlgorithm.equals(RlAlgorithm.DDPG)) {
                setActorNetwork(trainedArchitecture);
            } else if (rlAlgorithm.equals(RlAlgorithm.TD3)) {
                setActorNetwork(trainedArchitecture);
            }
        } else if (trainingConfiguration.isGanLearning()) {
            setGeneratorNetwork(trainedArchitecture);
        } else if (trainingConfiguration.isVaeLearning()) {
            setDecoderNetwork(trainedArchitecture);
        }
    }

    public void setActorNetwork(ArchitectureAdapter actorNetwork) {
        this.actorNetwork = actorNetwork;
        addTrainingComponent(ACTOR, actorNetwork);
    }

    public void setCriticNetwork(ArchitectureAdapter criticNetwork) {
        this.criticNetwork = criticNetwork;
        addTrainingComponent(CRITIC, criticNetwork);
    }

    public void setGeneratorNetwork(ArchitectureAdapter generatorNetwork) {
        this.generatorNetwork = generatorNetwork;
        addTrainingComponent(GENERATOR, generatorNetwork);
    }

    public void setDiscriminatorNetwork(ArchitectureAdapter discriminatorNetwork) {
        this.discriminatorNetwork = discriminatorNetwork;
        addTrainingComponent(DISCRIMINATOR, discriminatorNetwork);
    }

    public void setQNetwork(ArchitectureAdapter qNetwork) {
        this.qNetwork = qNetwork;
        addTrainingComponent(QNETWORK, qNetwork);
    }

    public void setDecoderNetwork(ArchitectureAdapter decoderNetwork) {
        this.decoderNetwork = decoderNetwork;
        addTrainingComponent(DECODER, decoderNetwork);
    }

    public void setEncoderNetwork(ArchitectureAdapter encoderNetwork) {
        this.encoderNetwork = encoderNetwork;
        addTrainingComponent(ENCODER, encoderNetwork);
    }

    public void setRewardFunction(EMAComponentInstanceSymbol rewardFunction) {
        this.rewardFunction = rewardFunction;
        addTrainingComponent(REWARD_FUNCTION, rewardFunction.getComponentType());
    }

    public void setPreprocessingComponentParameter(PreprocessingComponentParameterAdapter preprocessingComponentParameter) {
        this.preprocessingComponentParameter = preprocessingComponentParameter;
    }

    public void setRewardFunctionParameter(RewardFunctionParameterAdapter rewardFunctionParameter) {
        this.rewardFunctionParameter = rewardFunctionParameter;
    }

    public Map<String, ArchitectureComponent> getArchitectureComponents() {
        return architectureComponents;
    }

    public void addTrainingComponent(String name, EMAComponentSymbol component) {
        architectureComponents.put(name, schemalang.validation.model.MappingUtils.
                createArchitectureComponent(component));
    }

    public void addTrainingComponent(String name, ArchitectureAdapter architectureAdapter) {
        ArchitectureComponent component = createArchitectureComponent(architectureAdapter);
        architectureComponents.put(name, component);
    }

    public void addTrainingComponent(String name, ArchitectureComponent component) {
        architectureComponents.put(name, component);
    }
}