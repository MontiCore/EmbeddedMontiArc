/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain.cocos;

import de.monticore.lang.monticar.cnntrain._cocos.*;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

public class InterCocoTest extends AbstractCoCoTest {

    NNArchitecturerBuilder NNBuilder = new NNArchitecturerBuilder();

    @Test
    public void testValidTD3ActorCritic() {
        // given
        final NNArchitectureSymbol validActor = NNBuilder.getValidTrainedArchitecture();
        final NNArchitectureSymbol validCritic = NNBuilder.getValidCriticArchitecture();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            validActor, validCritic);

        // when
        checkValidTrainedArchitecture(configurationSymbol);
        checkValidCriticArchitecture(configurationSymbol);
    }

    @Test
    public void testValidDDPGActorCritic() {
        // given
        final NNArchitectureSymbol validActor = NNBuilder.getValidTrainedArchitecture();
        final NNArchitectureSymbol validCritic = NNBuilder.getValidCriticArchitecture();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "DdpgConfig",
            validActor, validCritic);

        // when
        checkValidTrainedArchitecture(configurationSymbol);
        checkValidCriticArchitecture(configurationSymbol);
    }

    @Test
    public void testValidDefaultGAN() {
        // given
        final NNArchitectureSymbol validGenerator = NNBuilder.getValidGenerator();
        final NNArchitectureSymbol validDiscriminator = NNBuilder.getValidDiscriminator();
        ConfigurationSymbol configurationSymbol = getDefaultGANConfigurationSymbolFrom("valid_tests", "DefaultGANConfig",
                validGenerator, validDiscriminator);

        // when
        checkValidGANArchitecture(configurationSymbol);
    }

    @Test
    public void testValidInfoGAN() {
        // given
        final NNArchitectureSymbol validGenerator = NNBuilder.getValidInfoGANGenerator();
        final NNArchitectureSymbol validDiscriminator = NNBuilder.getValidDiscriminatorWithQNet();
        final NNArchitectureSymbol validQNetwork = NNBuilder.getValidQNetwork();
        ConfigurationSymbol configurationSymbol = getInfoGANConfigurationSymbolFrom("valid_tests", "InfoGANConfig",
                validGenerator, validDiscriminator, validQNetwork);

        // when
        checkValidGANArchitecture(configurationSymbol);
    }

    @Test
    public void testInvalidTrainingArchitectureWithTwoInputs() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckTrainedRlNetworkHasExactlyOneInput();
        NNArchitectureSymbol nnWithTwoInputs = NNBuilder.getTrainedArchitectureWithTwoInputs();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests",
            "TD3Config", nnWithTwoInputs, NNBuilder.getValidCriticArchitecture());

        // when
        checkInvalidTrainedArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.TRAINED_ARCHITECTURE_ERROR));


    }

    @Test
    public void testInvalidTrainingArchitectureWithTwoOutputs() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckTrainedRlNetworkHasExactlyOneOutput();
        NNArchitectureSymbol nnWithTwoOutputs = NNBuilder.getTrainedArchitectureWithTwoOutputs();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests",
        "TD3Config", nnWithTwoOutputs, NNBuilder.getValidCriticArchitecture());

        // when
        checkInvalidTrainedArchitecture(configurationSymbol, cocoUUT, new ExpectedErrorInfo(1, ErrorCodes.TRAINED_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidActionDimensionUnequalToOUParameterDimension1() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckOUParameterDimensionEqualsActionDimension();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("invalid_cocos_tests",
        "UnequalOUDim1", NNBuilder.getValidTrainedArchitecture(), NNBuilder.getValidCriticArchitecture());

        // when
        checkInvalidTrainedArchitecture(
            configurationSymbol, cocoUUT, new ExpectedErrorInfo(3, ErrorCodes.TRAINED_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidActionDimensionUnequalToOUParameterDimension2() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckOUParameterDimensionEqualsActionDimension();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("invalid_cocos_tests",
        "UnequalOUDim2", NNBuilder.getValidTrainedArchitecture(), NNBuilder.getValidCriticArchitecture());

        // when
        checkInvalidTrainedArchitecture(
            configurationSymbol, cocoUUT, new ExpectedErrorInfo(2, ErrorCodes.TRAINED_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidActionDimensionUnequalToOUParameterDimension3() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckOUParameterDimensionEqualsActionDimension();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("invalid_cocos_tests",
        "UnequalOUDim3", NNBuilder.getValidTrainedArchitecture(), NNBuilder.getValidCriticArchitecture());

        // when
        checkInvalidTrainedArchitecture(
            configurationSymbol, cocoUUT, new ExpectedErrorInfo(1, ErrorCodes.TRAINED_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidCriticHasNotOneDimensionalOutput() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkHasExactlyAOneDimensionalOutput();
        NNArchitectureSymbol criticWithThreeDimensionalOutput = NNBuilder.getCriticWithThreeDimensionalOutput();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithThreeDimensionalOutput);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidCriticHasTwoOutputs() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkHasExactlyAOneDimensionalOutput();
        NNArchitectureSymbol criticWithThreeDimensionalOutput = NNBuilder.getCriticWithTwoOutputs();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithThreeDimensionalOutput);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidCriticHasDifferentStateDimensions() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkInputs();
        NNArchitectureSymbol criticWithDifferentDimensions = NNBuilder.getCriticWithDifferentStateDimensions();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithDifferentDimensions);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }
    
    @Test
    public void testInvalidCriticHasDifferentActionDimensions() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkInputs();
        NNArchitectureSymbol criticWithDifferentDimensions = NNBuilder.getCriticWithDifferentActionDimensions();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithDifferentDimensions);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidCriticHasDifferentStateTypes() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkInputs();
        NNArchitectureSymbol criticWithDifferentStateTypes = NNBuilder.getCriticWithDifferentStateTypes();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithDifferentStateTypes);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidCriticHasDifferentActionTypes() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkInputs();
        NNArchitectureSymbol criticWithDifferentActionTypes = NNBuilder.getCriticWithDifferentActionTypes();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithDifferentActionTypes);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidCriticHasDifferentStateRanges() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkInputs();
        NNArchitectureSymbol criticWithDifferentStateRanges = NNBuilder.getCriticWithDifferentStateRanges();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithDifferentStateRanges);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidCriticHasDifferentActionRanges() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckCriticNetworkInputs();
        NNArchitectureSymbol criticWithDifferentActionRanges = NNBuilder.getCriticWithDifferentActionRanges();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            NNBuilder.getValidTrainedArchitecture(), criticWithDifferentActionRanges);

        // when
        checkInvalidCriticArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.CRITIC_NETWORK_ERROR));
    }

    @Test
    public void testInvalidTrainedArchitectureWithMultidimensionalAction() {
        // given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckTrainedArchitectureHasVectorAction();
        NNArchitectureSymbol actorWithMultidimensionalAction = NNBuilder.getTrainedArchitectureWithMultidimensionalAction();
        NNArchitectureSymbol criticWithMultidimensionalAction = NNBuilder.getCriticWithMultidimensionalAction();
        ConfigurationSymbol configurationSymbol = getConfigurationSymbolFrom("valid_tests", "TD3Config",
            actorWithMultidimensionalAction, criticWithMultidimensionalAction);
        // when
        checkInvalidTrainedArchitecture(configurationSymbol, cocoUUT,
            new ExpectedErrorInfo(1, ErrorCodes.TRAINED_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidDiscriminatorQNetworkDependency() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckGANDiscriminatorQNetworkDependency();
        NNArchitectureSymbol gen = NNBuilder.getValidGenerator();
        NNArchitectureSymbol dis = NNBuilder.getValidDiscriminator();
        NNArchitectureSymbol qnet = NNBuilder.getValidQNetwork();
        ConfigurationSymbol configurationSymbol = getInfoGANConfigurationSymbolFrom("valid_tests", "InfoGANConfig",
                gen, dis, qnet);

        // when
        checkInvalidGANArchitecture(configurationSymbol, cocoUUT,
                new ExpectedErrorInfo(1, ErrorCodes.GAN_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidGeneratorQNetworkDependency() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckGANGeneratorQNetworkDependency();
        NNArchitectureSymbol gen = NNBuilder.getValidGenerator();
        NNArchitectureSymbol dis = NNBuilder.getValidDiscriminatorWithQNet();
        NNArchitectureSymbol qnet = NNBuilder.getValidQNetwork();
        ConfigurationSymbol configurationSymbol = getInfoGANConfigurationSymbolFrom("valid_tests", "InfoGANConfig",
                gen, dis, qnet);

        // when
        checkInvalidGANArchitecture(configurationSymbol, cocoUUT,
                new ExpectedErrorInfo(1, ErrorCodes.GAN_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidGeneratorHasMultipleOutputs() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckGANGeneratorHasOneOutput();
        NNArchitectureSymbol gen = NNBuilder.getInvalidGeneratorMultipleOutputs();
        NNArchitectureSymbol dis = NNBuilder.getValidDiscriminator();
        ConfigurationSymbol configurationSymbol = getDefaultGANConfigurationSymbolFrom("valid_tests", "DefaultGANConfig",
                gen, dis);

        // when
        checkInvalidGANArchitecture(configurationSymbol, cocoUUT,
                new ExpectedErrorInfo(1, ErrorCodes.GAN_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidGeneratorDiscriminatorDependency() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckGANGeneratorDiscriminatorDependency();
        NNArchitectureSymbol gen = NNBuilder.getValidGenerator();
        NNArchitectureSymbol dis = NNBuilder.getValidDiscriminatorDifferentInput();
        ConfigurationSymbol configurationSymbol = getDefaultGANConfigurationSymbolFrom("valid_tests", "DefaultGANConfig",
                gen, dis);

        // when
        checkInvalidGANArchitecture(configurationSymbol, cocoUUT,
                new ExpectedErrorInfo(1, ErrorCodes.GAN_ARCHITECTURE_ERROR));
    }

    @Test
    public void testInvalidQNetworkMultipleInputs() {
        //given
        CNNTrainConfigurationSymbolCoCo cocoUUT = new CheckGANQNetworkhasOneInput();
        NNArchitectureSymbol gen = NNBuilder.getValidGenerator();
        NNArchitectureSymbol dis = NNBuilder.getValidDiscriminatorWithQNet();
        NNArchitectureSymbol qnet = NNBuilder.getInvalidQNetworkMultipleInputs();
        ConfigurationSymbol configurationSymbol = getInfoGANConfigurationSymbolFrom("valid_tests", "InfoGANConfig",
                gen, dis, qnet);

        // when
        checkInvalidGANArchitecture(configurationSymbol, cocoUUT,
                new ExpectedErrorInfo(1, ErrorCodes.GAN_ARCHITECTURE_ERROR));
    }

    private ConfigurationSymbol getConfigurationSymbolFrom(final String modelPath, final String model,
                                                           final NNArchitectureSymbol actorArchitecture, final NNArchitectureSymbol criticArchitecture) {
        final ConfigurationSymbol configurationSymbol = getConfigurationSymbolByPath( modelPath, model);
        configurationSymbol.setTrainedArchitecture(actorArchitecture);
        configurationSymbol.setCriticNetwork(criticArchitecture);
        return configurationSymbol;
    }

    private ConfigurationSymbol getDefaultGANConfigurationSymbolFrom(final String modelPath, final String model,
                                                           final NNArchitectureSymbol genArchitecture, final NNArchitectureSymbol disArchitecture) {
        final ConfigurationSymbol configurationSymbol = getConfigurationSymbolByPath( modelPath, model);
        configurationSymbol.setTrainedArchitecture(genArchitecture);
        configurationSymbol.setDiscriminatorNetwork(disArchitecture);
        return configurationSymbol;
    }

    private ConfigurationSymbol getInfoGANConfigurationSymbolFrom(final String modelPath, final String model,
                                                              final NNArchitectureSymbol genArchitecture,
                                                              final NNArchitectureSymbol disArchitecture,
                                                              final NNArchitectureSymbol qnetArchitecture) {
            final ConfigurationSymbol configurationSymbol = getConfigurationSymbolByPath( modelPath, model);
        configurationSymbol.setTrainedArchitecture(genArchitecture);
        configurationSymbol.setDiscriminatorNetwork(disArchitecture);
        configurationSymbol.setQNetwork(qnetArchitecture);
        return configurationSymbol;
    }

    private ConfigurationSymbol getConfigurationSymbolByPath(final String modelPath, final String model) {
        return getCompilationUnitSymbol(modelPath, model).getConfiguration();
    }

    private enum CheckOption {
        TRAINED_ARCHITECTURE_COCOS,
        CRITIC_ARCHITECTURE_COCOS,
        GAN_ARCHITECTURE_COCOS,
    }

    private void checkInvalidArchitecture(
        final ConfigurationSymbol configurationSymbol,
        final CNNTrainConfigurationSymbolCoCo cocoUUT,
        final ExpectedErrorInfo expectedErrors,
        final CheckOption checkOption) {
        Log.getFindings().clear();

        if (checkOption.equals(CheckOption.TRAINED_ARCHITECTURE_COCOS)) {
            CNNTrainCocos.checkTrainedArchitectureCoCos(configurationSymbol);
        } else if(checkOption.equals(CheckOption.CRITIC_ARCHITECTURE_COCOS)) {
            CNNTrainCocos.checkCriticCocos(configurationSymbol);
        } else if(checkOption.equals(CheckOption.GAN_ARCHITECTURE_COCOS)) {
            CNNTrainCocos.checkGANCocos(configurationSymbol);
        }


        expectedErrors.checkExpectedPresent(Log.getFindings(), "Got no findings when checking all "
                + "cocos. Did you forget to add the new coco to MontiArcCocos?");
        Log.getFindings().clear();
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker().addCoCo(cocoUUT);
        checker.checkAll(configurationSymbol);
        expectedErrors.checkOnlyExpectedPresent(Log.getFindings(), "Got no findings when checking only "
                + "the given coco. Did you pass an empty coco checker?");
    }

    private void checkInvalidArchitectureOnlyCoCo(
            final ConfigurationSymbol configurationSymbol,
            final CNNTrainConfigurationSymbolCoCo cocoUUT,
            final ExpectedErrorInfo expectedErrors) {
        Log.getFindings().clear();

        Log.getFindings().clear();
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker().addCoCo(cocoUUT);
        checker.checkAll(configurationSymbol);
        expectedErrors.checkOnlyExpectedPresent(Log.getFindings(), "Got no findings when checking only "
                + "the given coco. Did you pass an empty coco checker?");
    }

    private void checkInvalidTrainedArchitecture(
        final ConfigurationSymbol configurationSymbol,
        final CNNTrainConfigurationSymbolCoCo cocoUUT,
        ExpectedErrorInfo expectedErrors) {
        checkInvalidArchitecture(configurationSymbol, cocoUUT, expectedErrors, CheckOption.TRAINED_ARCHITECTURE_COCOS);
    }

    private void checkInvalidCriticArchitecture(
        final ConfigurationSymbol configurationSymbol,
        final CNNTrainConfigurationSymbolCoCo cocoUUT,
        ExpectedErrorInfo expectedErrors) {
        checkInvalidArchitecture(configurationSymbol, cocoUUT, expectedErrors, CheckOption.CRITIC_ARCHITECTURE_COCOS);
    }

    private void checkValidTrainedArchitecture(final ConfigurationSymbol configurationSymbol) {
        Log.getFindings().clear();
        CNNTrainCocos.checkTrainedArchitectureCoCos(configurationSymbol);
        new ExpectedErrorInfo().checkOnlyExpectedPresent(Log.getFindings());
    }

    private void checkValidCriticArchitecture(final ConfigurationSymbol configurationSymbol) {
        Log.getFindings().clear();
        CNNTrainCocos.checkCriticCocos(configurationSymbol);
        new ExpectedErrorInfo().checkOnlyExpectedPresent(Log.getFindings());
    }

    private void checkValidGANArchitecture(final ConfigurationSymbol configurationSymbol) {
        Log.getFindings().clear();
        CNNTrainCocos.checkGANCocos(configurationSymbol);
        new ExpectedErrorInfo().checkOnlyExpectedPresent(Log.getFindings());
    }

    private void checkInvalidGANArchitecture(
            final ConfigurationSymbol configurationSymbol,
            final CNNTrainConfigurationSymbolCoCo cocoUUT,
            ExpectedErrorInfo expectedErrors) {
        checkInvalidArchitectureOnlyCoCo(configurationSymbol, cocoUUT, expectedErrors);
    }
}
