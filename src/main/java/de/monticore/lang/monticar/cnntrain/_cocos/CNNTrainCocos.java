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

import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainNode;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.se_rwth.commons.logging.Log;

public class CNNTrainCocos {

    public static CNNTrainCoCoChecker createChecker() {
        return new CNNTrainCoCoChecker()
                .addCoCo(new CheckEntryRepetition())
                .addCoCo(new CheckInteger())
                .addCoCo(new CheckFixTargetNetworkRequiresInterval())
                .addCoCo(new CheckReinforcementRequiresEnvironment())
                .addCoCo(new CheckLearningParameterCombination())
                .addCoCo(new CheckRosEnvironmentRequiresRewardFunction())
                .addCoCo(new CheckActorCriticRequiresCriticNetwork())
                .addCoCo(new CheckRlAlgorithmParameter())
                .addCoCo(new CheckDiscreteRLAlgorithmUsesDiscreteStrategy())
                .addCoCo(new CheckContinuousRLAlgorithmUsesContinuousStrategy())
                .addCoCo(new CheckRosEnvironmentHasOnlyOneRewardSpecification());
    }

    public static void checkAll(CNNTrainCompilationUnitSymbol compilationUnit){
        ASTCNNTrainNode node = (ASTCNNTrainNode) compilationUnit.getAstNode().get();
        int findings = Log.getFindings().size();
        createChecker().checkAll(node);
    }

    public static void checkTrainedArchitectureCoCos(final ConfigurationSymbol configurationSymbol) {
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker()
                .addCoCo(new CheckTrainedRlNetworkHasExactlyOneInput())
                .addCoCo(new CheckTrainedRlNetworkHasExactlyOneOutput());
        checker.checkAll(configurationSymbol);
    }

    public static void checkCriticCocos(final ConfigurationSymbol configurationSymbol) {
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker()
                .addCoCo(new CheckCriticNetworkHasExactlyAOneDimensionalOutput())
                .addCoCo(new CheckCriticNetworkInputs());
        checker.checkAll(configurationSymbol);
    }
}