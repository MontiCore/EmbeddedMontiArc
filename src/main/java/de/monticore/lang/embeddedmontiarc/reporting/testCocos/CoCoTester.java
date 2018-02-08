package de.monticore.lang.embeddedmontiarc.reporting.testCocos;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.cocos.EmbeddedMontiArcCoCos;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEmbeddedMontiArcMathNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.EmbeddedMontiArcMathCoCos;
import de.monticore.lang.embeddedmontiarc.reporting.cocos.AbstractCoCoTest;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.ASTHelper;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.CouldNotResolveException;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.ResolveSymbolException;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;
import de.monticore.lang.math.math._symboltable.MathLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Paths;
import java.util.stream.Collectors;


public class CoCoTester {
    private static boolean containsErrorCode(String errorCode) {
        return Log.getFindings().stream().map(s -> s.getMsg()).collect(Collectors.joining(" ")).contains(errorCode);
    }

    public TestResult testCoCos(String fileName){
        Log.enableFailQuick(false);
        Log.getFindings().clear();
        TestResult testResult = new TestResult(fileName);
        fileName = fileName.replace("\\","/");
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMAMCompilationUnit ast = null;
        try {
            testResult.addErrorMessage("[INFO] do Parser Test <br>=========================");
            for(Finding finding: Log.getFindings())
                testResult.addErrorMessage(finding.getMsg());
            ast = parser.parse(fileName).orElse(null);
            testResult.addErrorMessage(ast != null?"[INFO] Parser Test success<br>":"[ERROR] Parser Test failed");
        } catch (Exception e) {
            testResult.addErrorMessage("[ERROR] Parser Test failed");
            System.out.println("File \"" + fileName + "\" could not be parsed");
        }
        boolean parse = ast != null;
        if(!parse){
            for(Finding finding: Log.getFindings()){
                testResult.addErrorMessage(finding.getMsg());
            }
            String FileName = fileName.substring(fileName.replace("\\","/").lastIndexOf("/") + 1, fileName.length());
            testResult.setModelName(FileName);
        }

        String fileType = fileName.substring(fileName.lastIndexOf(".") + 1, fileName.length()).toUpperCase();
        testResult.setType(fileType);
        testResult.setParsed(parse?1:-1);

        if(parse) {

            String PackageName = Joiners.DOT.join(ast.getEMACompilationUnit().getPackage());
            String FileName    = fileName.substring(fileName.replace("\\","/").lastIndexOf("/") + 1, fileName.length());
            String modelPath   = fileName.substring(0, fileName.length() - (PackageName + "/" + FileName).length()); // package name + File name
            String modelName   = PackageName + "." + FileName.replace(".emam", "").replace(".ema", "");
            testResult.setModelName(modelName);

            ASTEmbeddedMontiArcNode astToTest = null;
            try {
                Log.getFindings().clear();
                testResult.addErrorMessage("[INFO] do Resolve Test<br>=========================");
                astToTest = ASTHelper.getAstNode(modelPath, modelName, fileType);
                testResult.setResolve(1);
                testResult.addErrorMessage("[INFO] Resolve Test success<br>");
            } catch (CouldNotResolveException e) {
                System.out.println("File \"" + modelName + "\" could not be resolved");
                testResult.setResolve(-1);
                testResult.addErrorMessage("[ERROR] Resolve Test failed");
                for(Finding finding: Log.getFindings())
                    testResult.addErrorMessage(finding.getMsg());

                return testResult;
            } catch (Exception e) {
                System.out.println("File \"" + modelName + "\" ERROR. Something went wrong");
                testResult.setResolve(-1);
                testResult.addErrorMessage("[ERROR] Something went wrong");
                return testResult;
            }
            testResult.addErrorMessage("[INFO] do CoCo Tests<br>=========================");
            Log.getFindings().clear();
            EmbeddedMontiArcMathCoCos.createChecker().checkAll(astToTest);
            for(Finding finding: Log.getFindings())
                testResult.addErrorMessage(finding.getMsg());
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

            if(fileType.equals("EMA") || fileType.equals("EMAM")){
                testResult.setComponentCapitalized(componentCapitalized?-1:1);
                testResult.setComponentInstanceNamesUnique(componentInstanceNamesUnique?-1:1);
                testResult.setComponentWithTypeParametersHasInstance(componentWithTypeParametersHasInstance?-1:1);
                testResult.setConnectorEndPointCorrectlyQualified(connectorEndPointCorrectlyQualified?-1:1);
                testResult.setDefaultParametersHaveCorrectOrder(defaultParametersHaveCorrectOrder?-1:1);
                testResult.setInPortUniqueSender(inPortUniqueSender?-1:1);
                testResult.setPackageLowerCase(packageLowerCase?-1:1);
                testResult.setParameterNamesUnique(parameterNamesUnique?-1:1);
                testResult.setPortTypeOnlyBooleanOrSIUnit(portTypeOnlyBooleanOrSIUnit?-1:1);
                testResult.setPortUsage(portUsage?-1:1);
                testResult.setReferencedSubComponentExists(referencedSubComponentExists?-1:1);
                testResult.setSimpleConnectorSourceExists(simpleConnectorSourceExists?-1:1);
                testResult.setSourceTargetNumberMatch(sourceTargetNumberMatch?-1:1);
                testResult.setSubComponentsConnected(subComponentsConnected?-1:1);
                testResult.setTopLevelComponentHasNoInstanceName(topLevelComponentHasNoInstanceName?-1:1);
                testResult.setTypeParameterNamesUnique(typeParameterNamesUnique?-1:1);
                testResult.setUniquePorts(uniquePorts?-1:1);
            }

            if(fileType.equals("EMAM"))
                testResult.setAtomicComponent(atomicComponent?-1:1);
        }
        return testResult;
    }
}
