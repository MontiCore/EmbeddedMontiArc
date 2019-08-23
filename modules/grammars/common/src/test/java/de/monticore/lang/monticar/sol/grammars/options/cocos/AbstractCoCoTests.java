/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Guice;
import de.monticore.lang.monticar.sol.grammars.options.OptionsModule;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options.optionstest._parser.OptionsTestParser;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;

public abstract class AbstractCoCoTests {
    protected final OptionsCoCoChecker checker;
    protected final OptionsTestParser parser;
    protected final OptionCoCo coCo;

    protected AbstractCoCoTests() {
        this.parser = new OptionsTestParser();
        this.checker = new OptionsCoCoChecker();
        this.coCo = Guice.createInjector(new OptionsModule()).getInstance(this.getContextCondition());

        this.coCo.registerTo(this.checker);
    }

    protected abstract Class<? extends OptionCoCo> getContextCondition();

    protected abstract int getExpectedViolations();

    @BeforeAll
    public static void beforeAll() {
        Log.enableFailQuick(false);
    }

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    protected ASTOption fetchAST(String folder) throws IOException {
        String className = this.getClass().getSimpleName().replace("CoCoTests", "");
        String filename = String.format("%s.test", className);
        File testFile = Paths.get("src/test/resources/cocos", folder, filename).toFile();

        return this.parser.parse_String(FileUtils.readFileToString(testFile, "UTF-8")).get();
    }

    @Test
    protected void testCheck() throws IOException {
        this.checker.checkAll(this.fetchAST("valid"));

        assertEquals(0, Log.getFindings().size(), "Number of violations does not match.");

        this.checker.checkAll(this.fetchAST("invalid"));

        assertEquals(this.getExpectedViolations(), Log.getFindings().size(), "Number of violations does not match.");
    }
}
