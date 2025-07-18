/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentModelLoader;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;

public abstract class AbstractCoCoTests {
    protected final EnvironmentCoCoChecker checker;
    protected final EnvironmentCoCo coCo;
    protected final EnvironmentModelLoader loader;

    protected AbstractCoCoTests() {
        Injector injector = Guice.createInjector(new EnvironmentModule());

        this.checker = new EnvironmentCoCoChecker();
        this.coCo = injector.getInstance(this.getContextCondition());
        this.loader = injector.getInstance(EnvironmentModelLoader.class);

        this.coCo.registerTo(this.checker);
    }

    protected abstract Class<? extends EnvironmentCoCo> getContextCondition();

    protected abstract int getExpectedViolations();

    @BeforeAll
    public static void beforeAll() {
        Log.enableFailQuick(false);
    }

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    protected ASTEnvironmentCompilationUnit fetchAST(String folder) {
        String className = this.getClass().getSimpleName().replace("CoCoTests", "");
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/cocos"));
        String qualifiedName = String.format("%s.%s", folder, className);

        return this.loader.loadModelsIntoScope(qualifiedName, modelPath).iterator().next();
    }

    @Test
    protected void testCheck() {
        this.checker.checkAll(this.fetchAST("valid"));

        assertEquals(0, Log.getFindings().size(), "Number of violations does not match.");

        this.checker.checkAll(this.fetchAST("invalid"));

        assertEquals(this.getExpectedViolations(), Log.getFindings().size(), "Number of violations does not match.");
    }
}
