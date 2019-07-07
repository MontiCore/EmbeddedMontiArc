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
package de.monticore.lang.monticar.cnntrain.cocos;

import de.monticore.lang.monticar.cnntrain._cocos.*;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.IOException;

public class AllCoCoTest extends AbstractCoCoTest{

    public AllCoCoTest() {
        Log.enableFailQuick(false);
    }

    @Test
    public void testValidCoCos() throws IOException {
        checkValid("valid_tests","SimpleConfig1");
        checkValid("valid_tests","SimpleConfig2");
        checkValid("valid_tests","FullConfig");
        checkValid("valid_tests","FullConfig2");
        checkValid("valid_tests", "ReinforcementConfig");
        checkValid("valid_tests", "ReinforcementConfig2");
        checkValid("valid_tests", "DdpgConfig");
        checkValid("valid_tests", "TD3Config");
        checkValid("valid_tests", "ReinforcementWithRosReward");
    }

    @Test
    public void testInvalidCoCos() throws IOException {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckEntryRepetition()),
                "invalid_cocos_tests", "EntryRepetition",
                new ExpectedErrorInfo(1, ErrorCodes.ENTRY_REPETITION_CODE));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckInteger()),
                "invalid_cocos_tests", "IntegerTest",
                new ExpectedErrorInfo(1, ErrorCodes.NOT_INTEGER_CODE));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckFixTargetNetworkRequiresInterval()),
                "invalid_cocos_tests", "FixTargetNetworkRequiresInterval1",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckFixTargetNetworkRequiresInterval()),
                "invalid_cocos_tests", "FixTargetNetworkRequiresInterval2",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination1",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination2",
                new ExpectedErrorInfo(3, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination3",
                new ExpectedErrorInfo(2, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination4",
                new ExpectedErrorInfo(5, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckReinforcementRequiresEnvironment()),
                "invalid_cocos_tests", "CheckReinforcementRequiresEnvironment",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRosEnvironmentRequiresRewardFunction()),
                "invalid_cocos_tests", "CheckRosEnvironmentRequiresRewardFunction",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter1",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter2",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter3",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckDiscreteRLAlgorithmUsesDiscreteStrategy()),
                "invalid_cocos_tests", "CheckDiscreteRLAlgorithmUsesDiscreteStrategy",
                new ExpectedErrorInfo(1, ErrorCodes.STRATEGY_NOT_APPLICABLE));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckContinuousRLAlgorithmUsesContinuousStrategy()),
                "invalid_cocos_tests", "CheckContinuousRLAlgorithmUsesContinuousStrategy",
                new ExpectedErrorInfo(1, ErrorCodes.STRATEGY_NOT_APPLICABLE));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRosEnvironmentHasOnlyOneRewardSpecification()),
                "invalid_cocos_tests", "CheckRosEnvironmentHasOnlyOneRewardSpecification",
                new ExpectedErrorInfo(1, ErrorCodes.CONTRADICTING_PARAMETERS));
    }
}
