package de.monticore.reporting.cocoReport;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.EmbeddedMontiArcMathCoCos;
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

    public CheckCoCoResult testCoCos(String fileName) {
        Log.enableFailQuick(false);
        Log.getFindings().clear();
        CheckCoCoResult testResult = new CheckCoCoResult(fileName);
        ASTHelper.setTestResultInfo(testResult);

        if (testResult.getParsed() == 1 && testResult.getResolved() == 1) {
            String fileType = testResult.getFileType();
            ASTEmbeddedMontiArcNode astToTest = testResult.getResolvedASTNode();

            testResult.addErrorMessage("[INFO] do CoCo Tests<br>=========================");
            Log.getFindings().clear();
            Log.enableFailQuick(false);
            EmbeddedMontiArcMathCoCos.createChecker().checkAll(astToTest);
            for (Finding finding : Log.getFindings())
                testResult.addErrorMessage("[WARNING] " + finding.toString());
            boolean componentCapitalized = containsErrorCode("0xAC004");
            boolean componentInstanceNamesUnique = containsErrorCode("0xAC010");
            boolean componentWithTypeParametersHasInstance = containsErrorCode("0x79C00");
            boolean connectorEndPointCorrectlyQualified = containsErrorCode("0xDB61C");
            boolean defaultParametersHaveCorrectOrder = containsErrorCode("0xAC005");
            boolean inPortUniqueSender = containsErrorCode("0x2BD7E");
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
            boolean atomicComponent = containsErrorCode("0x00000AE1");

            if (fileType.equals("EMA") || fileType.equals("EMAM")) {
                testResult.setComponentCapitalized(componentCapitalized ? -1 : 1);
                testResult.setComponentInstanceNamesUnique(componentInstanceNamesUnique ? -1 : 1);
                testResult.setComponentWithTypeParametersHasInstance(componentWithTypeParametersHasInstance ? -1 : 1);
                testResult.setConnectorEndPointCorrectlyQualified(connectorEndPointCorrectlyQualified ? -1 : 1);
                testResult.setDefaultParametersHaveCorrectOrder(defaultParametersHaveCorrectOrder ? -1 : 1);
                testResult.setInPortUniqueSender(inPortUniqueSender ? -1 : 1);
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

            if (fileType.equals("EMAM"))
                testResult.setAtomicComponent(atomicComponent ? -1 : 1);

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
}
