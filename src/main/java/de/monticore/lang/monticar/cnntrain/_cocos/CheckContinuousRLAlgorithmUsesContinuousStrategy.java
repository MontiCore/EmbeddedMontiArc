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
package de.monticore.lang.monticar.cnntrain._cocos;

import com.google.common.collect.ImmutableSet;
import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Set;

public class CheckContinuousRLAlgorithmUsesContinuousStrategy implements CNNTrainASTConfigurationCoCo{
    private static final Set<String> CONTINUOUS_STRATEGIES = ImmutableSet.<String>builder()
        .add("ornstein_uhlenbeck")
        .add("gaussian")
        .build();

    @Override
    public void check(ASTConfiguration node) {
        if (ASTConfigurationUtils.isContinuousAlgorithm(node)
            && ASTConfigurationUtils.hasStrategy(node)
            && ASTConfigurationUtils.getStrategyMethod(node).isPresent()) {
            final String usedStrategy = ASTConfigurationUtils.getStrategyMethod(node).get();
            if (!CONTINUOUS_STRATEGIES.contains(usedStrategy)) {
                Log.error("0" + ErrorCodes.STRATEGY_NOT_APPLICABLE + " Strategy " + usedStrategy + " used but" +
                 " continuous algorithm used.", node.get_SourcePositionStart());
            }
        }
    }
}