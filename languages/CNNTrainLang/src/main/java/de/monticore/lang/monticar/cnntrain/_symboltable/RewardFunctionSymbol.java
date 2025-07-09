/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnntrain.annotations.RewardFunctionParameter;
import de.monticore.symboltable.CommonSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 *
 */
public class RewardFunctionSymbol extends CommonSymbol {
    public static final RewardFunctionSymbolKind KIND = new RewardFunctionSymbolKind();

    private List<String> rewardFunctionComponentName;
    private RewardFunctionParameter rewardFunctionParameter;

    public RewardFunctionSymbol(String name) {
        super(name, KIND);
        rewardFunctionComponentName = new ArrayList<>();
    }

    protected void setRewardFunctionComponentName(List<String> rewardFunctionComponentNamePath) {
        this.rewardFunctionComponentName = Lists.newArrayList(rewardFunctionComponentNamePath);
    }

    public List<String> getRewardFunctionComponentName() {
        return Lists.newArrayList(rewardFunctionComponentName);
    }

    public void setRewardFunctionParameter(RewardFunctionParameter rewardFunctionParameter) {
        this.rewardFunctionParameter = rewardFunctionParameter;
    }

    public Optional<RewardFunctionParameter> getRewardFunctionParameter() {
        return Optional.ofNullable(rewardFunctionParameter);
    }
}
