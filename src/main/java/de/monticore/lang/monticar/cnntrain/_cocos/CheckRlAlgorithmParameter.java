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
import de.monticore.lang.monticar.cnntrain._ast.ASTEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTRLAlgorithmEntry;
import de.monticore.lang.monticar.cnntrain._symboltable.RLAlgorithm;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckRlAlgorithmParameter implements CNNTrainASTEntryCoCo {
    private final ParameterAlgorithmMapping parameterAlgorithmMapping;

    private boolean isDqn = true;
    private boolean isDdpg = true;
    private boolean isTd3 = true;

    RLAlgorithm algorithm;

    public CheckRlAlgorithmParameter() {
        parameterAlgorithmMapping = new ParameterAlgorithmMapping();
    }


    @Override
    public void check(ASTEntry node) {
        if (!parameterAlgorithmMapping.isReinforcementLearningParameter(node.getClass())) {
            return;
        }
        if (node instanceof ASTRLAlgorithmEntry) {
            ASTRLAlgorithmEntry algorithmEntry = (ASTRLAlgorithmEntry)node;
            if (algorithmEntry.getValue().isPresentDdpg()) {
                logWrongParameterIfCheckFails(isDdpg, node);
                isTd3 = false;
                isDqn = false;
            } else if(algorithmEntry.getValue().isPresentTdThree()) {
                logWrongParameterIfCheckFails(isTd3, node);
                isDdpg = false;
                isDqn = false;
            } else {
                logWrongParameterIfCheckFails(isDqn, node);
                isDdpg = false;
                isTd3 = false;
            }
        } else {
            final boolean isDdpgParameter = parameterAlgorithmMapping.isDdpgParameter(node.getClass());
            final boolean isDqnParameter = parameterAlgorithmMapping.isDqnParameter(node.getClass());
            final boolean isTd3Parameter = parameterAlgorithmMapping.isTd3Parameter(node.getClass());
            if (!isDdpgParameter) {
                isDdpg = false;
            }
            if (!isTd3Parameter) {
                isTd3 = false;
            }
            if (!isDqnParameter) {
                isDqn = false;
            }
        }
        logWrongParameterIfCheckFails(isDqn || isTd3 || isDdpg, node);
    }

    private void logWrongParameterIfCheckFails(final boolean condition, final ASTEntry node) {
        if (!condition) {
            Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER
                            + "Parameter " + node.getName() + " used but parameter is not for chosen algorithm.",
                    node.get_SourcePositionStart());
        }
    }
}