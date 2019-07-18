/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._parser.EnvironmentParser;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;

public abstract class AbstractCoCoTests {
    protected final EnvironmentCoCoChecker checker;
    protected final EnvironmentParser parser;

    protected AbstractCoCoTests() {
        this.parser = new EnvironmentParser();
        this.checker = new EnvironmentCoCoChecker();

        this.getContextCondition().registerTo(this.checker);
    }

    protected abstract EnvironmentCoCo getContextCondition();

    protected abstract String getExpectedErrorCode();

    protected abstract String getExpectedErrorMessage();

    protected abstract Object[] getMessageParameters();

    protected abstract int getExpectedViolations();

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    protected ASTEnvironmentCompilationUnit fetchAST(String folder) throws IOException {
        String className = this.getClass().getSimpleName().replace("CoCoTests", "");
        String filename = String.format("%s.ddf", className);
        File testFile = Paths.get("src/test/resources/cocos", folder, filename).toFile();
        String contents = FileUtils.readFileToString(testFile, StandardCharsets.UTF_8);

        return this.parser.parse_String(contents).get();
    }

    @Test
    protected void testGetErrorCode() {
        assertEquals(
                this.getExpectedErrorCode(),
                this.getContextCondition().getErrorCode(),
                "Error Codes do not match."
        );
    }

    @Test
    protected void testGetErrorMessage() {
        assertEquals(
                this.getExpectedErrorMessage(),
                this.getContextCondition().getErrorMessage(this.getMessageParameters()),
                "Error Messages do not match."
        );
    }

    @Test
    protected void testCheck() throws IOException {
        this.checker.checkAll(this.fetchAST("valid"));

        assertEquals(0, Log.getFindings().size(), "Number of violations does not match.");

        this.checker.checkAll(this.fetchAST("invalid"));

        assertEquals(this.getExpectedViolations(), Log.getFindings().size(), "Number of violations does not match.");
    }
}
