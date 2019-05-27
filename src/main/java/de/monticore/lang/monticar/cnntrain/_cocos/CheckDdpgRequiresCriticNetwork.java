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

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain._ast.ASTCriticNetworkEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTRLAlgorithmEntry;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckDdpgRequiresCriticNetwork implements CNNTrainASTConfigurationCoCo {

    @Override
    public void check(ASTConfiguration node) {
        boolean isDdpg = node.getEntriesList().stream()
            .anyMatch(e -> e instanceof ASTRLAlgorithmEntry
                && ((ASTRLAlgorithmEntry)e).getValue().isPresentDdpg());
        boolean hasCriticEntry = node.getEntriesList().stream()
            .anyMatch(e -> ((e instanceof ASTCriticNetworkEntry)
                && !((ASTCriticNetworkEntry)e).getValue().getNameList().isEmpty()));

        if (isDdpg && !hasCriticEntry) {
            ASTRLAlgorithmEntry algorithmEntry = node.getEntriesList().stream()
                    .filter(e -> e instanceof ASTRLAlgorithmEntry)
                    .map(e -> (ASTRLAlgorithmEntry)e)
                    .findFirst()
                    .orElseThrow(() -> new IllegalStateException("ASTRLAlgorithmEntry entry must be available"));
            Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING + " DDPG learning algorithm requires critc" +
             " network entry", algorithmEntry.get_SourcePositionStart());
        }
    }
}