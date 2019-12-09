/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.*;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionLanguage;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class SymbolTableTests {
    static IDESymbol root;

    @BeforeAll
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    static void beforeAll() {
        IDELanguage language = new IDELanguage();
        IDEModelLoader loader = new IDEModelLoader(language);
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/symboltable").toAbsolutePath());
        ResolvingConfiguration configuration = new ResolvingConfiguration();
        ModelingLanguageFamily family = new ModelingLanguageFamily();

        family.addModelingLanguage(language);
        family.addModelingLanguage(new ArtifactLanguage());
        family.addModelingLanguage(new OptionLanguage());

        GlobalScope scope = new GlobalScope(modelPath, family);

        Log.initDEBUG();
        Log.enableFailQuick(false);
        configuration.addDefaultFilters(language.getResolvingFilters());
        loader.loadModelsIntoScope("emastudio.EMAStudio", modelPath, scope, configuration);

        root = scope.<IDESymbol>resolve("emastudio.EMAStudio", IDESymbol.KIND).get();
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testIDESymbol() {
        assertEquals(1, root.getParentSymbols().size());
        assertEquals(1, root.getModuleTypeInclusionSymbols().size());
        assertEquals(1, root.getConfigurationTypeInclusions().size());
        assertEquals(1, root.getModuleTypeExclusionSymbols().size());
        assertEquals("https://hub.docker.com/some/awesome/image:latest", root.getRegistry().get());
        assertEquals("some/amazing/path", root.getBuildPath().get());
        assertEquals(CommonLiterals.ROOT, root.getBuildOrigin().get());
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testConfigurationTypeSymbol() {
        ConfigurationTypeSymbol configurationType = root.getConfigurationTypeInclusionSymbols().iterator().next();
        List<OptionSymbol> options = configurationType.getOptionSymbols();
        List<TaskSymbol> tasks = configurationType.getTaskSymbols();

        assertTrue(configurationType.isComponent());
        assertEquals(1, tasks.size());
        assertEquals(1, options.size());
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testModuleTypeSymbol() {
        ModuleTypeSymbol moduleType = root.getModuleTypeInclusionSymbols().iterator().next();
        List<ConfigurationSymbol> configurations = moduleType.getConfigurationSymbols();
        List<OptionFillSymbol> fills = configurations.get(0).getOptionFillSymbols();

        assertEquals("PacMan", moduleType.getLabel());
        assertEquals("some-icon-class", moduleType.getIcon().get());
        assertEquals("EMAStudio", moduleType.getCategory().get());
        assertEquals(1, configurations.size());
        assertEquals(2, fills.size());
    }
}
