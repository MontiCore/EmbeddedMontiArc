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

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.RLAlgorithm;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

/**
 *
 */
public class CheckTrainedRlNetworkHasExactlyOneInput implements CNNTrainConfigurationSymbolCoCo {
    @Override
    public void check(ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.isReinforcementLearningMethod()
                && configurationSymbol.getTrainedArchitecture().isPresent()) {
            final int numberOfInputs = configurationSymbol.getTrainedArchitecture().get().getInputs().size();
            if (numberOfInputs != 1) {
                final String networkName
                        = configurationSymbol.getEntry(ConfigEntryNameConstants.RL_ALGORITHM).getValue().getValue()
                        .equals(RLAlgorithm.DQN) ? "Q-Network" : "Actor-Network";
                Log.error("x0" + ErrorCodes.TRAINED_ARCHITECTURE_ERROR
                        + networkName + " " +configurationSymbol.getTrainedArchitecture().get().getName()
                        +" has " + numberOfInputs + " inputs but 1 is only allowed.", configurationSymbol.getSourcePosition());
            }
        }
    }
}