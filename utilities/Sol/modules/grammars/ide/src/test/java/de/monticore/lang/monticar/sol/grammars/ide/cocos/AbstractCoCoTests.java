/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.ide.IDEModule;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDECompilationUnit;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDEModelLoader;
import de.monticore.lang.monticar.sol.grammars.option.OptionModule;
import de.monticore.lang.monticar.sol.grammars.artifact.ArtifactModule;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;

public abstract class AbstractCoCoTests {
    protected final IDECoCoChecker checker;
    protected final IDECoCo coCo;
    protected final IDEModelLoader loader;

    protected AbstractCoCoTests() {
        Injector injector = Guice.createInjector(new IDEModule(), new OptionModule(), new ArtifactModule());

        this.checker = new IDECoCoChecker();
        this.coCo = injector.getInstance(this.getContextCondition());
        this.loader = injector.getInstance(IDEModelLoader.class);

        this.coCo.registerTo(this.checker);
    }

    protected abstract Class<? extends IDECoCo> getContextCondition();

    protected abstract List<Integer> getExpectedViolations();

    @BeforeAll
    public static void beforeAll() {
        // Log.initDEBUG();
        Log.enableFailQuick(false);
    }

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    protected ASTIDECompilationUnit fetchAST(String folder, String name) {
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/cocos"));
        String qualifiedName = String.format("%s.%s", folder, name);

        return this.loader.loadModelsIntoScope(qualifiedName, modelPath).iterator().next();
    }

    protected List<ASTIDECompilationUnit> fetchASTs(String folder) {
        return this.getTestCases().stream()
                .map(testCase -> this.fetchAST(folder, testCase))
                .collect(Collectors.toList());
    }

    @Test
    protected void testCheck() {
        List<ASTIDECompilationUnit> validASTs = this.fetchASTs("valid");
        List<ASTIDECompilationUnit> invalidASTs = this.fetchASTs("invalid");

        for (int i = 0; i < validASTs.size(); i++) {
            int expectedViolations = this.getExpectedViolations().get(i);
            ASTIDECompilationUnit validAST = validASTs.get(i);
            ASTIDECompilationUnit invalidAST = invalidASTs.get(i);

            Log.getFindings().clear();
            this.checker.checkAll(validAST);
            assertEquals(0, Log.getFindings().size(), "Number of violations does not match.");
            Log.getFindings().clear();
            this.checker.checkAll(invalidAST);
            assertEquals(expectedViolations, Log.getFindings().size(), "Number of violations does not match.");
        }
    }

    protected List<String> getTestCases() {
        return Collections.singletonList(this.getClass().getSimpleName().replace("CoCoTests", ""));
    }
}
