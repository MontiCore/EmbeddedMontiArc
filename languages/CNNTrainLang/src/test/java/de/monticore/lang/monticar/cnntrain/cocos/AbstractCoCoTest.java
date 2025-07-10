/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain.cocos;

import de.monticore.lang.monticar.cnntrain.AbstractSymtabTest;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainCompilationUnit;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainNode;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCoCoChecker;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainConfigurationSymbolChecker;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainConfigurationSymbolCoCo;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import static org.junit.Assert.*;


public class AbstractCoCoTest extends AbstractSymtabTest {

    private static final String MODEL_PATH = "src/test/resources/";

    protected static ASTCNNTrainCompilationUnit getAstNode(String modelPath, String model) {
        return (ASTCNNTrainCompilationUnit)getCompilationUnitSymbol(modelPath, model).getAstNode().get();
    }

    protected static ASTCNNTrainCompilationUnit getAstNodeFromCompilationUnit(CNNTrainCompilationUnitSymbol comp) {
        return (ASTCNNTrainCompilationUnit)comp.getAstNode().get();
    }

    protected static CNNTrainCompilationUnitSymbol getCompilationUnitSymbol(String modelPath, String model) {
        Scope symTab = createSymTab(MODEL_PATH + modelPath);
        CNNTrainCompilationUnitSymbol comp = symTab.<CNNTrainCompilationUnitSymbol> resolve(
                model, CNNTrainCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull("Could not resolve model " + model, comp);

        return comp;
    }

    /**
     * Checks all cocos on the given model. Don't use for writing new test cases, use checkValid and
     * checkInvalid instead.
     */
    @Deprecated
    protected static void runCheckerWithSymTab(String modelPath, String model) {
        Log.getFindings().clear();

        ASTCNNTrainCompilationUnit node = getAstNode(modelPath, model);

        CNNTrainCocos.createChecker().checkAll(node);
    }

    /**
     * Checks all cocos on the given node, and checks for absence of errors. Use this for checking
     * valid models.
     */
    protected static void checkValid(String modelPath, String model) {
        Log.getFindings().clear();
        CNNTrainCompilationUnitSymbol comp = getCompilationUnitSymbol(modelPath, model);
        CNNTrainCocos.createChecker().checkAll(getAstNodeFromCompilationUnit(comp));
        new ExpectedErrorInfo().checkOnlyExpectedPresent(Log.getFindings());
    }

    /**
     * Runs coco checks on the model with two different coco sets: Once with all cocos, checking that
     * the expected errors are present; once only with the given cocos, checking that no additional
     * errors are present.
     */
    protected static void checkInvalid(CNNTrainCoCoChecker cocos,
                                       String modelPath,
                                       String model,
                                       ExpectedErrorInfo expectedErrors) {

        // check whether all the expected errors are present when using all cocos
        Log.getFindings().clear();
        ASTCNNTrainCompilationUnit node = getAstNode(modelPath, model);
        CNNTrainCocos.createChecker().checkAll(node);
        expectedErrors.checkExpectedPresent(Log.getFindings(), "Got no findings when checking all "
                + "cocos. Did you forget to add the new coco to MontiArcCocos?");

        // check whether only the expected errors are present when using only the given cocos
        Log.getFindings().clear();
        cocos.checkAll(node);
        expectedErrors.checkOnlyExpectedPresent(Log.getFindings(), "Got no findings when checking only "
                + "the given coco. Did you pass an empty coco checker?");
    }

    protected static class ExpectedErrorInfo {
        private static final Pattern ERROR_CODE_PATTERN = Pattern.compile("x[0-9A-F]{5}");

        private int numExpectedFindings = 0;

        private HashSet<String> expectedErrorCodes = new HashSet<>();

        private Predicate<String> containsExpectedErrorCode;

        /**
         * Raises an error if the given error codes don't match the convention for error codes in test
         * cases (no leading zero, capital hexadecimal digits)
         */
        protected static void checkExpectedErrorCodes(String[] errorCodes) {

            for (String errorCode : errorCodes) {
                if (!ERROR_CODE_PATTERN.matcher(errorCode).matches()) {
                    Log.error(String.format(
                            "The given expected error code \"%s\" is not a valid error code (pattern: \"%s\")",
                            errorCode, ERROR_CODE_PATTERN.pattern()));
                }
            }
        }

        protected static Set<String> collectErrorCodes(String findings) {
            Matcher matcher = ERROR_CODE_PATTERN.matcher(findings);

            Set<String> errorCodes = new HashSet<>();
            while (matcher.find()) {
                errorCodes.add(matcher.group());
            }

            return errorCodes;
        }

        private void initContainsExpectedErrorCode() {
            containsExpectedErrorCode = new Predicate<String>() {

                @Override
                public boolean test(String s) {
                    for (String errorCode : expectedErrorCodes) {
                        if (s.contains(errorCode)) {
                            return true;
                        }
                    }

                    return false;
                }
            };
        }

        public ExpectedErrorInfo() {
            this(0);
        }

        public ExpectedErrorInfo(int numExpectedFindings, String... expectedErrorCodes) {
            checkExpectedErrorCodes(expectedErrorCodes);

            this.numExpectedFindings = numExpectedFindings;
            this.expectedErrorCodes.addAll(Arrays.asList(expectedErrorCodes));

            initContainsExpectedErrorCode();
        }

        private String concatenateFindings(List<Finding> findings) {
            return findings.stream().map(f -> f.buildMsg())
                    .collect(Collectors.joining("\n"));
        }

        public void checkExpectedPresent(List<Finding> findings, String emptyFindingsHint) {
            String findingsString = concatenateFindings(findings);

            if (findingsString.isEmpty()) {
                findingsString = emptyFindingsHint;
            }

            assertEquals(findingsString, numExpectedFindings,
                    findings.stream().map(f -> f.buildMsg()).filter(containsExpectedErrorCode).count());

            assertTrue(collectErrorCodes(findingsString).containsAll(expectedErrorCodes));
        }

        public void checkOnlyExpectedPresent(List<Finding> findings) {
            checkOnlyExpectedPresent(findings, "");
        }

        public void checkOnlyExpectedPresent(List<Finding> findings, String emptyFindingsHint) {
            checkExpectedPresent(findings, emptyFindingsHint);

            checkNoAdditionalErrorCodesPresent(concatenateFindings(findings));
        }

        private void checkNoAdditionalErrorCodesPresent(String findingsString) {
            Set<String> actualErrorCodes = collectErrorCodes(findingsString);

            // check whether there are unexpected error codes
            Set<String> unexpectedErrorCodes = new HashSet<>(actualErrorCodes);
            unexpectedErrorCodes.removeAll(expectedErrorCodes);

            assertEquals(findingsString, 0, unexpectedErrorCodes.size());
        }
    }
}
