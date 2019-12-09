/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.artifact.ArtifactModule;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifactCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactModelLoader;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
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
    protected final ArtifactCoCoChecker checker;
    protected final ArtifactCoCo coCo;
    protected final ArtifactModelLoader loader;

    protected AbstractCoCoTests() {
        Injector injector = Guice.createInjector(new ArtifactModule(), new EnvironmentModule(), new LanguageModule());

        this.checker = new ArtifactCoCoChecker();
        this.coCo = injector.getInstance(this.getContextCondition());
        this.loader = injector.getInstance(ArtifactModelLoader.class);

        this.coCo.registerTo(this.checker);
    }

    protected abstract Class<? extends ArtifactCoCo> getContextCondition();

    protected abstract int getExpectedViolations();

    @BeforeAll
    public static void beforeAll() {
        // Log.initDEBUG();
        Log.enableFailQuick(false);
    }

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    protected ASTArtifactCompilationUnit fetchAST(String folder, String name) {
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/cocos"));
        String qualifiedName = String.format("%s.%s", folder, name);

        return this.loader.loadModelsIntoScope(qualifiedName, modelPath).iterator().next();
    }

    protected List<ASTArtifactCompilationUnit> fetchASTs(String folder) {
        return this.getTestCases().stream()
                .map(testCase -> this.fetchAST(folder, testCase))
                .collect(Collectors.toList());
    }

    @Test
    protected void testCheck() {
        this.fetchASTs("valid").forEach(ast -> {
            this.checker.checkAll(ast);

            assertEquals(0, Log.getFindings().size(), "Number of violations does not match.");
        });

        this.fetchASTs("invalid").forEach(ast -> {
            Log.getFindings().clear();
            this.checker.checkAll(ast);

            assertEquals(this.getExpectedViolations(), Log.getFindings().size(), "Number of violations does not match.");
        });
    }

    protected List<String> getTestCases() {
        return Collections.singletonList(this.getClass().getSimpleName().replace("CoCoTests", ""));
    }
}
