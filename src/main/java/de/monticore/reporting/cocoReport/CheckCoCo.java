/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.*;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.*;
import de.monticore.lang.math._cocos.*;
import de.monticore.lang.monticar.cnnarch._cocos.*;
import de.monticore.lang.monticar.emadl._cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;

import de.monticore.reporting.helper.TimedTask;
import de.monticore.reporting.tools.ASTHelper;
import de.monticore.reporting.tools.CustomPrinter;
import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.ArrayUtils;

import java.lang.reflect.Field;
import java.util.concurrent.*;
import java.util.stream.Collectors;


public class CheckCoCo {
    private static boolean containsErrorCode(String errorCode) {
        return Log.getFindings().stream().map(s -> s.getMsg()).collect(Collectors.joining(" ")).contains(errorCode);
    }

    public CheckCoCoResult testCoCos(String fileName, int timeout, int coCoTimeOut) {
        Log.init();
        Log.enableFailQuick(false);
        Log.getFindings().clear();
        CheckCoCoResult testResult = new CheckCoCoResult(fileName);
        ASTHelper.setTestResultInfo(testResult, timeout);

        if (testResult.getParsed() == 1 && testResult.getResolved() == 1) {
            String fileType = testResult.getFileType();
            ASTEmbeddedMontiArcNode astToTest = testResult.getResolvedASTNode();

            testResult.addErrorMessage("[INFO] do CoCo Tests<br>=========================");
            checkCoCos(testResult, astToTest, fileType, coCoTimeOut);

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

    private void checkCoCos(CheckCoCoResult testResult, ASTEmbeddedMontiArcNode astToTest, String fileType, int timeout) {
        Object[] cocos = getCoCos(fileType);
        for (Object coco : cocos) {
            String cocoName = coco.getClass().getSimpleName();
            cocoName = cocoName.substring(0, 1).toLowerCase() + cocoName.substring(1);
            int result = checkCoCo(astToTest, coco, timeout);

            try {
                Field field = testResult.getClass().getDeclaredField(cocoName);
                for (Finding finding : Log.getFindings())
                    testResult.addErrorMessage("[WARNING] " + field.getName() + ": " + finding.toString());
                field.set(testResult, result);
            } catch (NoSuchFieldException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    private int checkCoCo(ASTEmbeddedMontiArcNode astToTest, Object coco, int timeout) {
        Callable<Integer> task = () -> {
            Log.getFindings().clear();
            Log.enableFailQuick(false);

            EMADLCoCoChecker checker = new EMADLCoCoChecker();
            addCoCo(checker, coco);
            checker.checkAll(astToTest);

            if (Log.getErrorCount() > 0) return -1;
            else return 1;
        };

        int res = 0;

        Future<Integer> future = TimedTask.executeTask(task, timeout);
        try {
            res = future.get();
        } catch (CancellationException ex) {
            res = -2;
        } catch (InterruptedException e) {
            res = -2;
        } catch (Throwable e) {
            Log.error(e.getMessage());
            res = -1;
        }
        future.cancel(true);
        return res;
    }

    private void addCoCo(EMADLCoCoChecker checker, Object coco) {
        if (coco instanceof MatrixAssignmentDeclarationCheck)
            checker.addCoCo((MatrixAssignmentDeclarationCheck) coco);
        else if (coco instanceof MatrixAssignmentCheck)
            checker.addCoCo((MatrixAssignmentCheck) coco);
        else if (coco instanceof ComponentCapitalized)
            checker.addCoCo((ComponentCapitalized) coco);
        else if (coco instanceof ComponentWithTypeParametersHasInstance)
            checker.addCoCo((ComponentWithTypeParametersHasInstance) coco);
        else if (coco instanceof ConnectorEndPointCorrectlyQualified)
            checker.addCoCo((ConnectorEndPointCorrectlyQualified) coco);
        else if (coco instanceof DefaultParametersHaveCorrectOrder)
            checker.addCoCo((DefaultParametersHaveCorrectOrder) coco);
        else if (coco instanceof InPortUniqueSender)
            checker.addCoCo((InPortUniqueSender) coco);
        else if (coco instanceof InRosPortRosSender)
            checker.addCoCo((InRosPortRosSender) coco);
        else if (coco instanceof OnlyIncomingPortIsConfig)
            checker.addCoCo((OnlyIncomingPortIsConfig) coco);
        else if (coco instanceof PackageLowerCase)
            checker.addCoCo((PackageLowerCase) coco);
        else if (coco instanceof ParameterNamesUnique)
            checker.addCoCo((ParameterNamesUnique) coco);
        else if (coco instanceof PortTypeOnlyBooleanOrSIUnit)
            checker.addCoCo((PortTypeOnlyBooleanOrSIUnit) coco);
        else if (coco instanceof PortUsage)
            checker.addCoCo((PortUsage) coco);
        else if (coco instanceof ReferencedSubComponentExists)
            checker.addCoCo((ReferencedSubComponentExists) coco);
        else if (coco instanceof SimpleConnectorSourceExists)
            checker.addCoCo((SimpleConnectorSourceExists) coco);
        else if (coco instanceof SubComponentsConnected)
            checker.addCoCo((SubComponentsConnected) coco);
        else if (coco instanceof TopLevelComponentHasNoInstanceName)
            checker.addCoCo((TopLevelComponentHasNoInstanceName) coco);
        else if (coco instanceof TypeParameterNamesUnique)
            checker.addCoCo((TypeParameterNamesUnique) coco);
        else if (coco instanceof UniquePorts)
            checker.addCoCo((UniquePorts) coco);
        else if (coco instanceof DynamicComponentDynamicBodyElements)
            checker.addCoCo((DynamicComponentDynamicBodyElements) coco);
        else if (coco instanceof NoDynamicNewComponentAndPort)
            checker.addCoCo((NoDynamicNewComponentAndPort) coco);
        else if (coco instanceof NoDynamicNewConnectsOutsideEventHandler)
            checker.addCoCo((NoDynamicNewConnectsOutsideEventHandler) coco);
        else if (coco instanceof AtomicComponentCoCo)
            checker.addCoCo((AtomicComponentCoCo) coco);
        else if (coco instanceof ReferencedSubComponentExistsEMAM)
            checker.addCoCo((ReferencedSubComponentExistsEMAM) coco);
        else if (coco instanceof CheckLayer)
            checker.addCoCo((CheckLayer) coco);
        else if (coco instanceof CheckRangeOperators)
            checker.addCoCo((CheckRangeOperators) coco);
//        else if (coco instanceof CheckVariableName)
//            checker.addCoCo((CNNArchSymbolCoCo) coco);
        else if (coco instanceof CheckLayerName)
            checker.addCoCo((CheckLayerName) coco);
        else if (coco instanceof CheckArgument)
            checker.addCoCo((CheckArgument) coco);
        else if (coco instanceof CheckLayerRecursion)
            checker.addCoCo((CheckLayerRecursion) coco);
        else if (coco instanceof EMADLASTBehaviorNameCoCo)
            checker.addCoCo((EMADLASTBehaviorNameCoCo) coco);
        else if (coco instanceof EMADLASTBehaviorEmbeddingCoCo)
            checker.addCoCo((EMADLASTBehaviorEmbeddingCoCo) coco);
    }

    private Object[] getCoCos(String fileType) {
        Object[] cocos = {};
        if (isMath(fileType))
            cocos = ArrayUtils.addAll(cocos, getMathCoCos());
        if (isEMA(fileType))
            cocos = ArrayUtils.addAll(cocos, getEMACoCos());
        if (isDynamic(fileType))
            cocos = ArrayUtils.addAll(cocos, getDynamicCoCos());
        if (isEMAM(fileType))
            cocos = ArrayUtils.addAll(cocos, getEMAMCoCos());
        if (isCNNArch(fileType))
            cocos = ArrayUtils.addAll(cocos, getCNNArchCoCos());
        if (isEMADL(fileType))
            cocos = ArrayUtils.addAll(cocos, getEMADLCoCos());
        return cocos;
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

    private Object[] getMathCoCos() {
        return new Object[]{
                new MatrixAssignmentDeclarationCheck(),
                new MatrixAssignmentCheck()
        };
    }

    private Object[] getEMACoCos() {
        return new Object[]{
                new ComponentCapitalized(),
                new ComponentWithTypeParametersHasInstance(),
                new ConnectorEndPointCorrectlyQualified(),
                new DefaultParametersHaveCorrectOrder(),
                new InPortUniqueSender(),
                new InRosPortRosSender(),
                new OnlyIncomingPortIsConfig(),
                new PackageLowerCase(),
                new ParameterNamesUnique(),
                new PortTypeOnlyBooleanOrSIUnit(),
                new PortUsage(),
                new ReferencedSubComponentExists(),
                new SimpleConnectorSourceExists(),
                new SourceTargetNumberMatch(),
                new SubComponentsConnected(),
                new TopLevelComponentHasNoInstanceName(),
                new TypeParameterNamesUnique(),
                new UniquePorts()
        };
    }

    private Object[] getDynamicCoCos() {
        return new Object[]{
                new DynamicComponentDynamicBodyElements(),
                new NoDynamicNewComponentAndPort(),
                new NoDynamicNewConnectsOutsideEventHandler()
        };
    }

    private Object[] getEMAMCoCos() {
        return new Object[]{
                new AtomicComponentCoCo(),
                new ReferencedSubComponentExistsEMAM()
        };
    }

    private Object[] getCNNArchCoCos() {
        return new Object[]{
                new CheckLayer(),
                new CheckRangeOperators(),
                new CheckVariableName(),
                new CheckLayerName(),
                new CheckArgument(),
                new CheckLayerRecursion()
        };
    }

    private Object[] getEMADLCoCos() {
        return new Object[]{
                (EMADLASTBehaviorNameCoCo) new CheckBehaviorName(),
                (EMADLASTBehaviorEmbeddingCoCo) new CheckBehaviorName()
        };
    }

    private boolean stringEqualsOr(String str, String... others) {
        for (String other : others)
            if (other.equals(str)) return true;
        return false;
    }
}
