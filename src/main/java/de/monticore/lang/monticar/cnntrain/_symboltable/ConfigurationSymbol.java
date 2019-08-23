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

    public void setCriticNetwork(NNArchitectureSymbol criticNetwork) {
        this.criticNetwork = criticNetwork;
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

    public boolean hasCritic() {
        return getEntryMap().containsKey(CRITIC);
    }

    public Optional<String> getCriticName() {
        if (!hasCritic()) {
            return Optional.empty();
        }

        final Object criticNameValue = getEntry(CRITIC).getValue().getValue();
        assert criticNameValue instanceof String;
        return Optional.of((String)criticNameValue);
    }
}
