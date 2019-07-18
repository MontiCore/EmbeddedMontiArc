/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.CommonScopeSpanningSymbol;

import java.util.*;

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
        return this.entryMap.containsKey("learning_method")
                ? (LearningMethod)this.entryMap.get("learning_method").getValue().getValue() : LearningMethod.SUPERVISED;
    }

    public boolean isReinforcementLearningMethod() {
        return getLearningMethod().equals(LearningMethod.REINFORCEMENT);
    }

    public boolean hasCritic() {
        return getEntryMap().containsKey("critic");
    }

    public Optional<String> getCriticName() {
        if (!hasCritic()) {
            return Optional.empty();
        }

        final Object criticNameValue = getEntry("critic").getValue().getValue();
        assert criticNameValue instanceof String;
        return Optional.of((String)criticNameValue);
    }
}