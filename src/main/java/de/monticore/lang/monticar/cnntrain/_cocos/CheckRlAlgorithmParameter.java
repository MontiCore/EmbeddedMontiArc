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

import de.monticore.lang.monticar.cnntrain._ast.ASTEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTRLAlgorithmEntry;
import de.monticore.lang.monticar.cnntrain._symboltable.RLAlgorithm;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckRlAlgorithmParameter implements CNNTrainASTEntryCoCo {
    private final ParameterAlgorithmMapping parameterAlgorithmMapping;

    boolean algorithmKnown;
    RLAlgorithm algorithm;

    public CheckRlAlgorithmParameter() {
        parameterAlgorithmMapping = new ParameterAlgorithmMapping();
        algorithmKnown = false;
        algorithm = null;
    }


    @Override
    public void check(ASTEntry node) {
        final boolean isDdpgParameter = parameterAlgorithmMapping.isDdpgParameter(node.getClass());
        final boolean isDqnParameter = parameterAlgorithmMapping.isDqnParameter(node.getClass());

        if (node instanceof ASTRLAlgorithmEntry) {
            ASTRLAlgorithmEntry algorithmEntry = (ASTRLAlgorithmEntry)node;
            if (algorithmEntry.getValue().isPresentDdpg()) {
                setAlgorithmToDdpg(node);
            } else {
                setAlgorithmToDqn(node);
            }
        } else {
            if (isDdpgParameter && !isDqnParameter) {
                setAlgorithmToDdpg(node);
            } else if (!isDdpgParameter && isDqnParameter) {
                setAlgorithmToDqn(node);
            }
        }
    }

    private void logErrorIfAlgorithmIsDqn(final ASTEntry node) {
        if (algorithmKnown && algorithm.equals(RLAlgorithm.DQN)) {
            Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER
                    + " DDPG Parameter " + node.getName() + " used but algorithm is " + algorithm + ".",
                    node.get_SourcePositionStart());
        }
    }

    private void setAlgorithmToDdpg(final ASTEntry node) {
        logErrorIfAlgorithmIsDqn(node);
        algorithmKnown = true;
        algorithm = RLAlgorithm.DDPG;
    }

    private void setAlgorithmToDqn(final ASTEntry node) {
        logErrorIfAlgorithmIsDdpg(node);
        algorithmKnown = true;
        algorithm = RLAlgorithm.DQN;
    }

    private void logErrorIfAlgorithmIsDdpg(final ASTEntry node) {
        if (algorithmKnown && algorithm.equals(RLAlgorithm.DDPG)) {
            Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER
                    + " DQN Parameter " + node.getName() + " used but algorithm is " + algorithm + ".",
                    node.get_SourcePositionStart());
        }
    }
}