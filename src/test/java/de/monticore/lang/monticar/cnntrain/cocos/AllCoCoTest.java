/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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
    public void testValidSimpleConfig1() {
        checkValid("valid_tests","SimpleConfig1");
    }

    @Test
    public void testValidSimpleConfig2() {
        checkValid("valid_tests","SimpleConfig2");
    }

    @Test
    public void testValidFullConfig() {
        checkValid("valid_tests","FullConfig");
    }

    @Test
    public void testValidFullConfig2() {
        checkValid("valid_tests","FullConfig2");
    }

	@Test
    public void testValidFlowNetEPEConfig() {
        checkValid("valid_tests","FlowNetEPEConfig");
    }
	
    @Test
    public void testValidReinforcementConfig() {
        checkValid("valid_tests","ReinforcementConfig");
    }

    @Test
    public void testValidReinforcementConfig2() {
        checkValid("valid_tests","ReinforcementConfig2");
    }

    @Test
    public void testValidDdpgConfig() {
        checkValid("valid_tests","DdpgConfig");
    }

    @Test
    public void testValidTD3Config() {
        checkValid("valid_tests","TD3Config");
    }

    @Test
    public void testValidReinforcementWithRosReward() throws IOException {
        checkValid("valid_tests", "ReinforcementWithRosReward");
    }

    @Test
    public void testValidDefaultGANConfig() { checkValid("valid_tests", "DefaultGANConfig"); }

    @Test
    public void testValidInfoGANDefaultGANConfig() { checkValid("valid_tests", "InfoGANConfig"); }

    @Test
    public void testValidImageGANConfig() { checkValid("valid_tests", "ImageGANConfig"); }

    @Test
    public void testInvalidEntryRepetition() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckEntryRepetition()),
                "invalid_cocos_tests", "EntryRepetition",
                new ExpectedErrorInfo(1, ErrorCodes.ENTRY_REPETITION_CODE));
    }

    @Test
    public void testInvalidIntegerTest() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckInteger()),
                "invalid_cocos_tests", "IntegerTest",
                new ExpectedErrorInfo(1, ErrorCodes.NOT_INTEGER_CODE));
    }

    @Test
    public void testInvalidFixTargetNetworkRequiresInterval1() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckFixTargetNetworkRequiresInterval()),
                "invalid_cocos_tests", "FixTargetNetworkRequiresInterval1",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidFixTargetNetworkRequiresInterval2() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckFixTargetNetworkRequiresInterval()),
                "invalid_cocos_tests", "FixTargetNetworkRequiresInterval2",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidCheckLearningParameterCombination1() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination1",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckLearningParameterCombination2() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination2",
                new ExpectedErrorInfo(3, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckLearningParameterCombination3() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination3",
                new ExpectedErrorInfo(2, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckLearningParameterCombination4() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckLearningParameterCombination()),
                "invalid_cocos_tests", "CheckLearningParameterCombination4",
                new ExpectedErrorInfo(5, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckReinforcementRequiresEnvironment() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckReinforcementRequiresEnvironment()),
                "invalid_cocos_tests", "CheckReinforcementRequiresEnvironment",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidCheckRosEnvironmentRequiresRewardFunction() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRosEnvironmentRequiresRewardFunction()),
                "invalid_cocos_tests", "CheckRosEnvironmentRequiresRewardFunction",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidCheckRLAlgorithmParameter1() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter1",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckRLAlgorithmParameter2() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter2",
                new ExpectedErrorInfo(2, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckRLAlgorithmParameter3() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter3",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckRLAlgorithmParameter4() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRlAlgorithmParameter()),
                "invalid_cocos_tests", "CheckRLAlgorithmParameter4",
                new ExpectedErrorInfo(1, ErrorCodes.UNSUPPORTED_PARAMETER));
    }

    @Test
    public void testInvalidCheckDiscreteRLAlgorithmUsesDiscreteStrategy() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckDiscreteRLAlgorithmUsesDiscreteStrategy()),
                "invalid_cocos_tests", "CheckDiscreteRLAlgorithmUsesDiscreteStrategy",
                new ExpectedErrorInfo(1, ErrorCodes.STRATEGY_NOT_APPLICABLE));
    }

    @Test
    public void testInvalidCheckContinuousRLAlgorithmUsesContinuousStrategy() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckContinuousRLAlgorithmUsesContinuousStrategy()),
                "invalid_cocos_tests", "CheckContinuousRLAlgorithmUsesContinuousStrategy",
                new ExpectedErrorInfo(1, ErrorCodes.STRATEGY_NOT_APPLICABLE));
    }

    @Test
    public void testInvalidCheckRosEnvironmentHasOnlyOneRewardSpecification() {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRosEnvironmentHasOnlyOneRewardSpecification()),
                "invalid_cocos_tests", "CheckRosEnvironmentHasOnlyOneRewardSpecification",
                new ExpectedErrorInfo(1, ErrorCodes.CONTRADICTING_PARAMETERS));
    }

    @Test
    public void testInvalidCheckReinforcementLearningEntryIsSet () {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckRLParameterOnlyWithLearningMethodSet()),
                "invalid_cocos_tests", "CheckRLParameterOnlyWithLearningMethodSet",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidGeneratorLossTargetNameDependency () {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckGeneratorLossTargetNameDependency()),
                "invalid_cocos_tests", "CheckGeneratorLossTargetNameDependency",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidConstraintDistributionQNetworkDependency () {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckConstraintDistributionQNetworkDependency()),
                "invalid_cocos_tests", "CheckConstraintDistributionQNetworkDependency",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidConstraintLossQNetworkDependency () {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckConstraintLossesQNetworkDependency()),
                "invalid_cocos_tests", "CheckConstraintLossQNetworkDependency",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidInputDistributionDependency () {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckNoiseInputDistributionDependency()),
                "invalid_cocos_tests", "CheckInputDistributionDependency",
                new ExpectedErrorInfo(1, ErrorCodes.REQUIRED_PARAMETER_MISSING));
    }

    @Test
    public void testInvalidNoiseInputMissing () {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckNoiseInputMissing()),
                "invalid_cocos_tests", "CheckInputMissing",
                new ExpectedErrorInfo(1, ErrorCodes.MISSING_PARAMETER_VALUE_CODE));
    }
}
