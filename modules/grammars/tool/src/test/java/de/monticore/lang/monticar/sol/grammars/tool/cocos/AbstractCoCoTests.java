/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.tool.ToolModule;
import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTToolCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.ToolModelLoader;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public abstract class AbstractCoCoTests {
    protected final ToolCoCoChecker checker;
    protected final ToolCoCo coCo;
    protected final ToolModelLoader loader;

    protected AbstractCoCoTests() {
        Injector injector = Guice.createInjector(new ToolModule());

        this.checker = new ToolCoCoChecker();
        this.coCo = injector.getInstance(this.getContextCondition());
        this.loader = injector.getInstance(ToolModelLoader.class);

        this.coCo.registerTo(this.checker);
    }

    protected abstract Class<? extends ToolCoCo> getContextCondition();

    protected abstract int getExpectedViolations();

    @BeforeAll
    public static void beforeAll() {
        Log.enableFailQuick(false);
    }

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    protected ASTToolCompilationUnit fetchAST(String folder) {
        String className = this.getClass().getSimpleName().replace("CoCoTests", "");

        return this.fetchAST(folder, className);
    }

    protected ASTToolCompilationUnit fetchAST(String folder, String name) {
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/cocos"));
        String qualifiedName = String.format("%s.%s", folder, name);

        return this.loader.loadModelsIntoScope(qualifiedName, modelPath).iterator().next();
    }

    protected List<ASTToolCompilationUnit> fetchASTs(String folder) {
        List<ASTToolCompilationUnit> asts = new ArrayList<>();

        asts.add(this.fetchAST(folder));
        this.getAdditionalTestCases().forEach(name -> {
            asts.add(this.fetchAST(folder, name));
        });

        return asts;
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

    protected List<String> getAdditionalTestCases() {
        return new ArrayList<>();
    }
}
