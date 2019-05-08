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
