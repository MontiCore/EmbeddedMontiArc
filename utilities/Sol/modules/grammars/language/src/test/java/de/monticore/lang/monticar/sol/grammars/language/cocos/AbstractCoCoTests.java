/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageModelLoader;
import de.monticore.lang.monticar.sol.grammars.option.OptionModule;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;

public abstract class AbstractCoCoTests {
    protected final LanguageCoCoChecker checker;
    protected final LanguageCoCo coCo;
    protected final LanguageModelLoader loader;

    protected AbstractCoCoTests() {
        Injector injector = Guice.createInjector(new LanguageModule(), new OptionModule());

        this.checker = new LanguageCoCoChecker();
        this.coCo = injector.getInstance(this.getContextCondition());
        this.loader = injector.getInstance(LanguageModelLoader.class);

        this.coCo.registerTo(this.checker);
    }

    protected abstract Class<? extends LanguageCoCo> getContextCondition();

    protected abstract int getExpectedViolations();

    @BeforeAll
    public static void beforeAll() {
        Log.enableFailQuick(false);
    }

    @BeforeEach
    public void before() {
        Log.getFindings().clear();
    }

    protected ASTLanguageCompilationUnit fetchAST(String folder) {
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
