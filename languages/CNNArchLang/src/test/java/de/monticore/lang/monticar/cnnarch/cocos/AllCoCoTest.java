/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.cocos;

import de.monticore.lang.monticar.cnnarch._cocos.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static de.monticore.lang.monticar.cnnarch.ParserTest.ENABLE_FAIL_QUICK;

public class AllCoCoTest extends AbstractCoCoTest {
    String baseDir="src/test/resources";

    public AllCoCoTest() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testValidCoCos(){
        checkValid("architectures", "ResNeXt50");
        checkValid("architectures", "ResNet152");
        checkValid("architectures", "Alexnet");
        checkValid("architectures", "ResNet34");
        checkValid("architectures", "SequentialAlexnet");
        checkValid("architectures", "ThreeInputCNN_M14");
        checkValid("architectures", "VGG16");
        checkValid("architectures", "ShowAttendTell");
        checkValid("valid_tests", "ArgumentSequenceTest");
        checkValid("valid_tests", "Fixed_Alexnet");
        checkValid("valid_tests", "Fixed_ThreeInputCNN_M14");
        checkValid("valid_tests", "ThreeInputCNN_M14_alternative");
        checkValid("valid_tests", "Alexnet_alt");
        checkValid("valid_tests", "SimpleNetworkIdentity");
        checkValid("valid_tests", "SimpleLoadNetwork");
        checkValid("valid_tests", "SimpleNetworkSoftmax");
        checkValid("valid_tests", "SimpleNetworkSigmoid");
        checkValid("valid_tests", "SimpleNetworkLinear");
        checkValid("valid_tests", "SimpleNetworkRelu");
        checkValid("valid_tests", "SimpleNetworkTanh");
        checkValid("valid_tests", "ResNeXt50_alt");
        checkValid("valid_tests", "Alexnet_alt2");
        checkValid("valid_tests", "MultipleStreams");
        checkValid("valid_tests", "RNNencdec");
        checkValid("valid_tests", "RNNsearch");
        checkValid("valid_tests", "RNNtest");
        checkValid("valid_tests", "EpisodicMemoryNetwork");
        checkValid("valid_tests", "LargeMemoryNetwork");
        checkValid("valid_tests", "Small3DGan");
    }

    @Test
    public void testIllegalIONames(){
        checkInvalid(
                new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckVariableDeclarationName()),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "IllegalIOName",
                new ExpectedErrorInfo(2, ErrorCodes.ILLEGAL_NAME));
    }

    @Test
    public void testUnknownMethod(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckLayer()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "UnknownMethod",
                new ExpectedErrorInfo(1, ErrorCodes.UNKNOWN_LAYER));
    }

    @Test
    public void testDuplicatedNames(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckParameterName()).addCoCo(new CheckLayerName()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "DuplicatedNames",
                new ExpectedErrorInfo(2, ErrorCodes.DUPLICATED_NAME));
    }

    @Test
    public void testDuplicatedIONames(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckVariableDeclarationName()),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "DuplicatedIONames",
                new ExpectedErrorInfo(1, ErrorCodes.DUPLICATED_NAME));
    }

    @Test
    public void testUnknownParameterName(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckExpressions()),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "UnknownParameterName",
                new ExpectedErrorInfo(1, ErrorCodes.UNKNOWN_PARAMETER_NAME));
    }

    @Test
    public void testUnknownVariableName(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckVariableName()),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "UnknownVariableName",
                new ExpectedErrorInfo(2, ErrorCodes.UNKNOWN_VARIABLE_NAME));
    }

    @Test
    public void testDuplicatedArgument(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckLayer()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "DuplicatedArgument",
                new ExpectedErrorInfo(1, ErrorCodes.DUPLICATED_ARG));
    }

    @Test
    public void testWrongArgument(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckArgument()).addCoCo(new CheckLayer()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "WrongArgument",
                new ExpectedErrorInfo(4, ErrorCodes.UNKNOWN_ARGUMENT, ErrorCodes.MISSING_ARGUMENT));
    }

    @Test
    public void testAdaNetToManyLayer(){
        checkInvalid(
                new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckAdaNetTooManyLayers()),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "checkAdaNetToManyLayers",
                new ExpectedErrorInfo(1, ErrorCodes.ADANET_TOO_MANY_ADANET_LAYER));
    }

    @Test
    public void testAdaIllegalArch(){
        checkInvalid(
                new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckAdaNetMalFormedArchitecture()),
                "invalid_tests", "checkAdaNetIllegalArch",
                new ExpectedErrorInfo(1, ErrorCodes.ADANET_ILLEGAL_ARCH));
    }

    @Test
    public void testInvalidRecursion(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckLayerRecursion()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "InvalidRecursion",
                new ExpectedErrorInfo(1, ErrorCodes.RECURSION_ERROR));
    }

    @Test
    public void testArgumentConstraintTest1(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "ArgumentConstraintTest1",
                new ExpectedErrorInfo(1, ErrorCodes.ILLEGAL_ASSIGNMENT));
    }

    @Test
    public void testArgumentConstraintTest2(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "ArgumentConstraintTest2",
                new ExpectedErrorInfo(1, ErrorCodes.ILLEGAL_ASSIGNMENT));
    }

    @Test
    public void testWrongRangeOperator(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckRangeOperators()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "WrongRangeOperator",
                new ExpectedErrorInfo(2, ErrorCodes.DIFFERENT_RANGE_OPERATORS));
    }

    @Test
    public void testArgumentConstraintTest3(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "ArgumentConstraintTest3",
                new ExpectedErrorInfo(1, ErrorCodes.ILLEGAL_ASSIGNMENT));
    }

    @Test
    public void testArgumentConstraintTest4(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "ArgumentConstraintTest4",
                new ExpectedErrorInfo(1, ErrorCodes.ILLEGAL_ASSIGNMENT));
    }

    @Test
    public void testArgumentConstraintTest5(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "ArgumentConstraintTest5",
                new ExpectedErrorInfo(1, ErrorCodes.ILLEGAL_ASSIGNMENT));
    }

    @Test
    public void testArgumentConstraintTest6(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "ArgumentConstraintTest6",
                new ExpectedErrorInfo(1, ErrorCodes.ILLEGAL_ASSIGNMENT));
    }

    @Test
    public void testMissingArgument(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckLayer()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "MissingArgument",
                new ExpectedErrorInfo(3, ErrorCodes.MISSING_ARGUMENT));
    }

    @Test
    public void testIllegalName(){
        checkInvalid(new CNNArchCoCoChecker().addCoCo(new CheckParameterName()).addCoCo(new CheckLayerName()),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                "invalid_tests", "IllegalName",
                new ExpectedErrorInfo(2, ErrorCodes.ILLEGAL_NAME));
    }

    @Test
    public void testInvalidInputShape(){
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckElementInputs()),
                "invalid_tests", "InvalidInputShape",
                new ExpectedErrorInfo(2, ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE));
    }

    @Test
    public void testWrongIOType() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckElementInputs()),
                "invalid_tests", "WrongIOType",
                new ExpectedErrorInfo(1, ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN));
    }

    @Test
    public void testInvalidIOShape1() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckIOType()),
                "invalid_tests", "InvalidIOShape1",
                new ExpectedErrorInfo(1, ErrorCodes.INVALID_IO_TYPE));
    }

    @Test
    public void testInvalidIOShape2() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckIOType()),
                "invalid_tests", "InvalidIOShape2",
                new ExpectedErrorInfo(2, ErrorCodes.INVALID_IO_TYPE));
    }

    @Test
    public void testNotIOArray() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckIOAccessAndIOMissing()),
                "invalid_tests", "NotIOArray",
                new ExpectedErrorInfo(2, ErrorCodes.INVALID_ARRAY_ACCESS));
    }

    @Test
    public void testMissingIO2() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckIOAccessAndIOMissing()),
                "invalid_tests", "MissingIO2",
                new ExpectedErrorInfo(2, ErrorCodes.MISSING_IO));
    }

    @Test
    public void testInvalidArrayAccessValue() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckIOAccessAndIOMissing()),
                "invalid_tests", "InvalidArrayAccessValue",
                new ExpectedErrorInfo(1, ErrorCodes.INVALID_ARRAY_ACCESS));
    }

    @Test
    public void testMissingMerge() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckElementInputs()),
                "invalid_tests", "MissingMerge",
                new ExpectedErrorInfo(2, ErrorCodes.MISSING_MERGE));
    }

    @Test
    public void testOutputWrittenToMultipleTimes() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckIOAccessAndIOMissing()),
                "invalid_tests", "OutputWrittenToMultipleTimes",
                new ExpectedErrorInfo(1, ErrorCodes.OUTPUT_WRITTEN_TO_MULTIPLE_TIMES));
    }

    @Test
    public void testUnrollInputsTooMany() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckUnrollInputsOutputsTooMany()),
                "invalid_tests", "UnrollInputsTooMany",
                new ExpectedErrorInfo(1, ErrorCodes.UNROLL_INPUTS_TOO_MANY));
    }

    @Test
    public void testEpisodicMemoryInvalidPlacement() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckEpisodicMemoryLayer()),
                "invalid_tests", "EpisodicMemoryInvalidPlacement",
                new ExpectedErrorInfo(2, ErrorCodes.INVALID_EPISODIC_MEMORY_LAYER_PLACEMENT));
    }

    @Test
    public void testLargeMemoryInvalidParameterCombination() {
        checkInvalid(new CNNArchCoCoChecker(),
                new CNNArchSymbolCoCoChecker(),
                new CNNArchSymbolCoCoChecker().addCoCo(new CheckLargeMemoryLayer()),
                "invalid_tests", "LargeMemoryInvalidParameterCombination",
                new ExpectedErrorInfo(1, ErrorCodes.INVALID_LARGE_MEMORY_LAYER_PARAMETERS));
    }
}

