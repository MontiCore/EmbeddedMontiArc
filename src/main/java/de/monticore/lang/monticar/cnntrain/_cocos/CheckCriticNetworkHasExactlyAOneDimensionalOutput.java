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
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 *
 */
public class CheckCriticNetworkHasExactlyAOneDimensionalOutput implements CNNTrainConfigurationSymbolCoCo {

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.getCriticNetwork().isPresent()) {
            NNArchitectureSymbol criticNetwork = configurationSymbol.getCriticNetwork().get();

            if (criticNetwork.getOutputs().size() > 1) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " The critic network has more than one outputs", criticNetwork.getSourcePosition());
            }
            final String outputName = criticNetwork.getOutputs().get(0);
            List<Integer> dimensions = criticNetwork.getDimensions().get(outputName);

            if (dimensions.size() != 1 || dimensions.get(0) != 1) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR + " The output " + outputName
                        + " of critic network is not a one-dimensional vector", configurationSymbol.getSourcePosition());
            }
        }
    }
}
