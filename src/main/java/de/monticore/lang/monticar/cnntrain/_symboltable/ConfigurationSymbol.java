/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.CommonScopeSpanningSymbol;

import java.util.*;

import static de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants.*;

public class ConfigurationSymbol extends CommonScopeSpanningSymbol {

    private Map<String, EntrySymbol> entryMap = new HashMap<>();
    private OptimizerSymbol optimizer;
    private OptimizerSymbol criticOptimizer;
    private LossSymbol loss;
    private RewardFunctionSymbol rlRewardFunctionSymbol;
    private PreprocessingComponentSymbol preprocessingComponentSymbol;
    private NNArchitectureSymbol trainedArchitecture;
    private NNArchitectureSymbol criticNetwork;
    private NNArchitectureSymbol discriminatorNetwork;
    private NNArchitectureSymbol qNetwork;

    public static final ConfigurationSymbolKind KIND = new ConfigurationSymbolKind();

    public ConfigurationSymbol()  {
        super("", KIND);
        rlRewardFunctionSymbol = null;
        preprocessingComponentSymbol = null;
        trainedArchitecture = null;
    }

    public OptimizerSymbol getOptimizer() {
        return optimizer;
    }

    public void setOptimizer(OptimizerSymbol optimizer) {
        this.optimizer = optimizer;
    }

    public void setCriticOptimizer(OptimizerSymbol criticOptimizer) {
        this.criticOptimizer = criticOptimizer;
    }

    public Optional<OptimizerSymbol> getCriticOptimizer() {
        return Optional.ofNullable(criticOptimizer);
    }

    public LossSymbol getLoss() {
        return loss;
    }

    public void setLoss(LossSymbol loss) {
        this.loss = loss;
    }

    protected void setRlRewardFunction(RewardFunctionSymbol rlRewardFunctionSymbol) {
        this.rlRewardFunctionSymbol = rlRewardFunctionSymbol;
    }

    public Optional<RewardFunctionSymbol> getRlRewardFunction() {
        return Optional.ofNullable(this.rlRewardFunctionSymbol);
    }

    public void setPreprocessingComponent(PreprocessingComponentSymbol preprocessingComponentSymbol) {
        this.preprocessingComponentSymbol = preprocessingComponentSymbol;
    }

    public Optional<PreprocessingComponentSymbol> getPreprocessingComponent() {
        return Optional.ofNullable(this.preprocessingComponentSymbol);
    }

    public boolean hasPreprocessor() {
        return this.preprocessingComponentSymbol != null;
    }

    public Optional<NNArchitectureSymbol> getTrainedArchitecture() {
        return Optional.ofNullable(trainedArchitecture);
    }

    public void setTrainedArchitecture(NNArchitectureSymbol trainedArchitecture) {
        this.trainedArchitecture = trainedArchitecture;
    }

    public Optional<NNArchitectureSymbol> getCriticNetwork() {
        return Optional.ofNullable(criticNetwork);
    }

    public Optional<NNArchitectureSymbol> getDiscriminatorNetwork() {
        return Optional.ofNullable(discriminatorNetwork);
    }

    public Optional<NNArchitectureSymbol> getQNetwork() {
        return Optional.ofNullable(qNetwork);
    }

    public void setCriticNetwork(NNArchitectureSymbol criticNetwork) {
        this.criticNetwork = criticNetwork;
    }

    public void setDiscriminatorNetwork(NNArchitectureSymbol discriminatorNetwork) {
        this.discriminatorNetwork = discriminatorNetwork;
    }

    public void setQNetwork(NNArchitectureSymbol qNetwork) {
        this.qNetwork = qNetwork;
    }

    public Map<String, EntrySymbol> getEntryMap() {
        return entryMap;
    }

    public EntrySymbol getEntry(String name){
        return getEntryMap().get(name);
    }

    public LearningMethod getLearningMethod() {
        return this.entryMap.containsKey(LEARNING_METHOD)
                ? (LearningMethod)this.entryMap.get(LEARNING_METHOD).getValue().getValue() : LearningMethod.SUPERVISED;
    }

    public boolean isReinforcementLearningMethod() {
        return getLearningMethod().equals(LearningMethod.REINFORCEMENT);
    }

    public boolean isGAN() {
        return getLearningMethod().equals(LearningMethod.GAN);
    }

    public boolean hasCritic() {
        return getEntryMap().containsKey(CRITIC);
    }

    public boolean hasDiscriminator() {
        return getEntryMap().containsKey(DISCRIMINATOR_NAME);
    }

    public boolean hasQNetwork() {
        return getEntryMap().containsKey(QNETWORK_NAME);
    }

    public Optional<String> getCriticName() {
        if (!hasCritic()) {
            return Optional.empty();
        }

        final Object criticNameValue = getEntry(CRITIC).getValue().getValue();
        assert criticNameValue instanceof String;
        return Optional.of((String)criticNameValue);
    }

    public Optional<String> getDiscriminatorName() {
        if (!hasDiscriminator()) {
            return Optional.empty();
        }

        final Object discriminatorNameValue = getEntry(DISCRIMINATOR_NAME).getValue().getValue();
        assert discriminatorNameValue instanceof String;
        return Optional.of((String)discriminatorNameValue);
    }

    public Optional<String> getQNetworkName() {
        if (!hasQNetwork()) {
            return Optional.empty();
        }

        final Object qnetNameValue = getEntry(QNETWORK_NAME).getValue().getValue();
        assert qnetNameValue instanceof String;
        return Optional.of((String)qnetNameValue);
    }
}
