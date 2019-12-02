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
    private NNArchitectureSymbol trainedArchitecture;
    private NNArchitectureSymbol criticNetwork;
    private NNArchitectureSymbol discriminatorNetwork;

    public static final ConfigurationSymbolKind KIND = new ConfigurationSymbolKind();

    public ConfigurationSymbol()  {
        super("", KIND);
        rlRewardFunctionSymbol = null;
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

    public void setCriticNetwork(NNArchitectureSymbol criticNetwork) {
        this.criticNetwork = criticNetwork;
    }

    public void setDiscriminatorNetwork(NNArchitectureSymbol discriminatorNetwork) {
        this.discriminatorNetwork = discriminatorNetwork;
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

    public boolean hasPreprocessor() {
        return getEntryMap().containsKey(PREPROCESSING_NAME);
    }

    public boolean hasCritic() {
        return getEntryMap().containsKey(CRITIC);
    }

    public boolean hasDiscriminator() {
        return getEntryMap().containsKey(DISCRIMINATOR_NAME);
    }

    public Optional<String> getCriticName() {
        if (!hasCritic()) {
            return Optional.empty();
        }

        final Object criticNameValue = getEntry(CRITIC).getValue().getValue();
        assert criticNameValue instanceof String;
        return Optional.of((String)criticNameValue);
    }

    public Optional<String> getPreprocessingName() {
        if (!hasPreprocessor()) {
            return Optional.empty();
        }

        final Object preprocessingNameValue = getEntry(PREPROCESSING_NAME).getValue().getValue();
        assert preprocessingNameValue instanceof String;
        return Optional.of((String)preprocessingNameValue);
    }

    public Optional<String> getDiscriminatorName() {
        if (!hasDiscriminator()) {
            return Optional.empty();
        }

        final Object discriminatorNameValue = getEntry(DISCRIMINATOR_NAME).getValue().getValue();
        assert discriminatorNameValue instanceof String;
        return Optional.of((String)discriminatorNameValue);
    }
}