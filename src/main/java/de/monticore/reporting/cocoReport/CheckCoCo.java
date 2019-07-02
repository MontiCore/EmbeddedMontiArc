package de.monticore.reporting.cocoReport;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.*;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.*;
import de.monticore.lang.math._cocos.*;
import de.monticore.lang.monticar.cnnarch._cocos.*;
import de.monticore.lang.monticar.emadl._cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;

import de.monticore.reporting.tools.ASTHelper;
import de.monticore.reporting.tools.CustomPrinter;
import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.util.stream.Collectors;


public class CheckCoCo {
    private static boolean containsErrorCode(String errorCode) {
        return Log.getFindings().stream().map(s -> s.getMsg()).collect(Collectors.joining(" ")).contains(errorCode);
    }

    public CheckCoCoResult testCoCos(String fileName, int timeout) {
        Log.init();
        Log.enableFailQuick(false);
        Log.getFindings().clear();
        CheckCoCoResult testResult = new CheckCoCoResult(fileName);
        ASTHelper.setTestResultInfo(testResult, timeout);

        if (testResult.getParsed() == 1 && testResult.getResolved() == 1) {
            String fileType = testResult.getFileType();
            ASTEmbeddedMontiArcNode astToTest = testResult.getResolvedASTNode();

            testResult.addErrorMessage("[INFO] do CoCo Tests<br>=========================");
            Log.getFindings().clear();
            Log.enableFailQuick(false);
            //EmbeddedMontiArcMathCoCos.createChecker().checkAll(astToTest);
            createChecker(fileType).checkAll(astToTest);
            for (Finding finding : Log.getFindings())
                testResult.addErrorMessage("[WARNING] " + finding.toString());

            // Math
            // MatrixAssignmentDeclarationCheck Error Code Missing
            // MatrixAssignmentCheck Error Code Missing
            boolean matrixAssignmentDeclarationCheck = containsErrorCode("0x00000");
            boolean matrixAssignmentCheck = containsErrorCode("0x00000");
            // EMA
            boolean componentCapitalized = containsErrorCode("0xAC004");
            boolean componentWithTypeParametersHasInstance = containsErrorCode("0x79C00");
            boolean connectorEndPointCorrectlyQualified = containsErrorCode("0xDB61C");
            boolean defaultParametersHaveCorrectOrder = containsErrorCode("0xAC005");
            boolean inPortUniqueSender = containsErrorCode("0x2BD7E");
            boolean inRosPortRosSender = containsErrorCode("0x23a0d") || containsErrorCode("0x31f6e") || containsErrorCode("0x3830a");
            boolean onlyIncomingPortIsConfig = containsErrorCode("0x7FF02");
            boolean packageLowerCase = containsErrorCode("0xAC003");
            boolean parameterNamesUnique = containsErrorCode("0xC4A61");
            boolean portTypeOnlyBooleanOrSIUnit = containsErrorCode("0xAE753");
            boolean portUsage = containsErrorCode("0xAC006") || containsErrorCode("0xAC007");
            boolean referencedSubComponentExists = containsErrorCode("0x069B7");
            boolean simpleConnectorSourceExists = containsErrorCode("0x9AF6C") || containsErrorCode("0xBEA8B") || containsErrorCode("0xF4D71");
            boolean sourceTargetNumberMatch = containsErrorCode("0xJK901");
            boolean subComponentsConnected = containsErrorCode("0xAC008") || containsErrorCode("0xAC009");
            boolean topLevelComponentHasNoInstanceName = containsErrorCode("0xE51E8") || containsErrorCode("0x3F207");
            boolean typeParameterNamesUnique = containsErrorCode("0x35F1A");
            boolean uniquePorts = containsErrorCode("0xAC002");
            // Dynamics
            boolean dynamicComponentDynamicBodyElements = containsErrorCode("0xAD002");
            boolean noDynamicNewComponentAndPort = containsErrorCode("0xAD004") || containsErrorCode("0xAD005");
            boolean noDynamicNewConnectsOutsideEventHandler = containsErrorCode("0xAD003") || containsErrorCode("0xAD003");
            // EMAM
            boolean atomicComponent = containsErrorCode("0x00000AE1");
            boolean referencedSubComponentExistsEMAM = containsErrorCode("0x069B7");
            // CNN-Arch
            boolean checkLayer = containsErrorCode("0x03031") || containsErrorCode("0x33585") || containsErrorCode("0x06021");
            boolean checkRangeOperators = containsErrorCode("0xA8289");
            boolean checkVariableName = containsErrorCode("0x93567") || containsErrorCode("0x91569");
            boolean checkLayerName = containsErrorCode("0x93567") || containsErrorCode("0x91569");
            boolean checkArgument = containsErrorCode("0x92527");
            boolean checkLayerRecursion = containsErrorCode("0x25833");
            // EMADL
            // CheckBehaviorName Error Code Missing
            boolean checkBehaviorName = containsErrorCode("0x00000");

            // Math
            if (isMath(fileType)) {
                testResult.setMatrixAssignmentDeclarationCheck(matrixAssignmentDeclarationCheck ? -1 : 1);
                testResult.setMatrixAssignmentCheck(matrixAssignmentCheck ? -1 : 1);
            }
            // EMA
            if (isEMA(fileType)) {
                testResult.setComponentCapitalized(componentCapitalized ? -1 : 1);
                testResult.setComponentWithTypeParametersHasInstance(componentWithTypeParametersHasInstance ? -1 : 1);
                testResult.setConnectorEndPointCorrectlyQualified(connectorEndPointCorrectlyQualified ? -1 : 1);
                testResult.setDefaultParametersHaveCorrectOrder(defaultParametersHaveCorrectOrder ? -1 : 1);
                testResult.setInPortUniqueSender(inPortUniqueSender ? -1 : 1);
                testResult.setInRosPortRosSender(inRosPortRosSender ? -1 : 1);
                testResult.setOnlyIncomingPortIsConfig(onlyIncomingPortIsConfig ? -1 : 1);
                testResult.setPackageLowerCase(packageLowerCase ? -1 : 1);
                testResult.setParameterNamesUnique(parameterNamesUnique ? -1 : 1);
                testResult.setPortTypeOnlyBooleanOrSIUnit(portTypeOnlyBooleanOrSIUnit ? -1 : 1);
                testResult.setPortUsage(portUsage ? -1 : 1);
                testResult.setReferencedSubComponentExists(referencedSubComponentExists ? -1 : 1);
                testResult.setSimpleConnectorSourceExists(simpleConnectorSourceExists ? -1 : 1);
                testResult.setSourceTargetNumberMatch(sourceTargetNumberMatch ? -1 : 1);
                testResult.setSubComponentsConnected(subComponentsConnected ? -1 : 1);
                testResult.setTopLevelComponentHasNoInstanceName(topLevelComponentHasNoInstanceName ? -1 : 1);
                testResult.setTypeParameterNamesUnique(typeParameterNamesUnique ? -1 : 1);
                testResult.setUniquePorts(uniquePorts ? -1 : 1);
            }
            // Dynamics
            if (isDynamic(fileType)) {
                testResult.setDynamicComponentDynamicBodyElements(dynamicComponentDynamicBodyElements ? -1 : 1);
                testResult.setNoDynamicNewComponentAndPort(noDynamicNewComponentAndPort ? -1 : 1);
                testResult.setNoDynamicNewConnectsOutsideEventHandler(noDynamicNewConnectsOutsideEventHandler ? -1 : 1);
            }
            // EMAM
            if (isEMAM(fileType)) {
                testResult.setAtomicComponent(atomicComponent ? -1 : 1);
                testResult.setReferencedSubComponentExistsEMAM(referencedSubComponentExistsEMAM ? -1 : 1);
            }
            // CNNArch
            if (isCNNArch(fileType)) {
                testResult.setCheckLayer(checkLayer ? -1 : 1);
                testResult.setCheckRangeOperators(checkRangeOperators ? -1 : 1);
                testResult.setCheckVariableName(checkVariableName ? -1 : 1);
                testResult.setCheckLayerName(checkLayerName ? -1 : 1);
                testResult.setCheckArgument(checkArgument ? -1 : 1);
                testResult.setCheckLayerRecursion(checkLayerRecursion ? -1 : 1);
            }
            // EMADL
            if (isEMADL(fileType)) {
                testResult.setCheckBehaviorName(checkBehaviorName?-1:1);
            }

            if (testResult.isValid()) {
                testResult.addErrorMessage("[INFO] All CoCo tests successful <br>");
                CustomPrinter.println("SUCCESS");
            } else {
                testResult.addErrorMessage("[ERROR] Some CoCo test failed <br>");
                CustomPrinter.println("ERROR. Some CoCo test failed");
            }
        }
        return testResult;
    }

    private EMADLCoCoChecker createChecker(String fileType) {
        EMADLCoCoChecker checker = new EMADLCoCoChecker();
        if (isMath(fileType))
            checker = addMathCoCos(checker);
        if (isEMA(fileType))
            checker = addEMACoCos(checker);
//        if (isDynamic(fileType))
//            checker = addDynamicCoCos(checker);
        if (isEMAM(fileType))
            checker = addEMAMCoCos(checker);
//        if (isCNNArch(fileType))
//            checker = addCNNArchCoCos(checker);
//        if (isEMADL(fileType))
//            checker = addEMADLCoCos(checker);
        return checker;
    }

    private boolean isMath(String fileType) {
        return stringEqualsOr(fileType, "M", "EMAM", "EMADL");
    }

    private boolean isEMA(String fileType) {
        return stringEqualsOr(fileType, "EMA", "EMAM", "EMADL");
    }

    private boolean isDynamic(String fileType) {
        return stringEqualsOr(fileType, "EMAM", "EMADL");
    }

    private boolean isEMAM(String fileType) {
        return stringEqualsOr(fileType, "EMAM", "EMADL");
    }

    private boolean isCNNArch(String fileType) {
        return stringEqualsOr(fileType, "EMADL");
    }

    private boolean isEMADL(String fileType) {
        return stringEqualsOr(fileType, "EMADL");
    }

    private EMADLCoCoChecker addMathCoCos(EMADLCoCoChecker checker) {
        return checker
                .addCoCo(new MatrixAssignmentDeclarationCheck())
                .addCoCo(new MatrixAssignmentCheck());
    }

    private EMADLCoCoChecker addEMACoCos(EMADLCoCoChecker checker) {
        return checker
                .addCoCo(new ComponentCapitalized())
                .addCoCo(new ComponentWithTypeParametersHasInstance())
                .addCoCo(new ConnectorEndPointCorrectlyQualified())
                .addCoCo(new DefaultParametersHaveCorrectOrder())
                .addCoCo(new InPortUniqueSender())
                .addCoCo(new InRosPortRosSender())
                .addCoCo(new OnlyIncomingPortIsConfig())
                .addCoCo(new PackageLowerCase())
                .addCoCo(new ParameterNamesUnique())
                .addCoCo(new PortTypeOnlyBooleanOrSIUnit())
                .addCoCo(new PortUsage())
                .addCoCo(new ReferencedSubComponentExists())
                .addCoCo(new SimpleConnectorSourceExists())
                .addCoCo(new SourceTargetNumberMatch())
                .addCoCo(new SubComponentsConnected())
                .addCoCo(new TopLevelComponentHasNoInstanceName())
                .addCoCo(new TypeParameterNamesUnique())
                .addCoCo(new UniquePorts());
    }

    private EMADLCoCoChecker addDynamicCoCos(EMADLCoCoChecker checker) {
        return checker
                .addCoCo(new DynamicComponentDynamicBodyElements())
                .addCoCo(new NoDynamicNewComponentAndPort())
                .addCoCo(new NoDynamicNewConnectsOutsideEventHandler());
    }

    private EMADLCoCoChecker addEMAMCoCos(EMADLCoCoChecker checker) {
        return checker = checker
                .addCoCo(new AtomicComponentCoCo())
                .addCoCo(new ReferencedSubComponentExistsEMAM());
    }

    private EMADLCoCoChecker addCNNArchCoCos(EMADLCoCoChecker checker) {
        return checker = checker
                .addCoCo(new CheckLayer())
                .addCoCo(new CheckRangeOperators())
                .addCoCo(new CheckVariableName())
                .addCoCo(new CheckLayerName())
                .addCoCo(new CheckArgument())
                .addCoCo(new CheckLayerRecursion());
    }

    private EMADLCoCoChecker addEMADLCoCos(EMADLCoCoChecker checker) {
        return checker
                .addCoCo((EMADLASTBehaviorNameCoCo) new CheckBehaviorName())
                .addCoCo((EMADLASTBehaviorEmbeddingCoCo) new CheckBehaviorName());
    }

    private boolean stringEqualsOr(String str, String... others) {
        for (String other : others)
            if (other.equals(str)) return true;
        return false;
    }
}
