/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.cocos.EmbeddedMontiArcDynamicCoCos;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicLanguage;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventModelLoader;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.function.Predicate;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

public class AbstractTest {

    public static final boolean ENABLE_FAIL_QUICK = false;
    protected String MODEL_PATH = "src/test/resources/test/embeddedmontiarcdynamic/cocos/";

    public AbstractTest() {
        this("src/test/resources/test/embeddedmontiarcdynamic/cocos/");
    }

    public AbstractTest(String modelPath) {
        MODEL_PATH = modelPath;
    }

    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new EmbeddedMontiArcDynamicLanguage());
        fam.addModelingLanguage(new EventLanguage());

        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }

    @Before
    public void setUp() {
        // ensure an empty log
        Log.init();
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    protected void checkValid(String modelPath, String model) {
        Log.getFindings().clear();
        EmbeddedMontiArcDynamicCoCos.createChecker().checkAll(getAstNode(modelPath, model));
        new ExpectedErrorInfo().checkOnlyExpectedPresent(Log.getFindings());
    }

    protected void checkValid(String modelPath, String model, int numExpectedFindings, String... expectedErrorCodes) {
        Log.getFindings().clear();
        EmbeddedMontiArcDynamicCoCos.createChecker().checkAll(getAstNode(modelPath, model));
        new ExpectedErrorInfo(numExpectedFindings, expectedErrorCodes).checkOnlyExpectedPresent(Log.getFindings());
    }

    protected ASTEmbeddedMontiArcNode getAstNode(String modelPath, String model) {

        Scope symTab = createSymTab(MODEL_PATH + modelPath + "/");
        EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(
                model, EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("Could not resolve model " + model, comp);

        return (ASTEmbeddedMontiArcNode) comp.getAstNode().get();
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

