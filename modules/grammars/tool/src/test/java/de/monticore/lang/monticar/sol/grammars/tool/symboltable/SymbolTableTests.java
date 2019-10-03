/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.*;
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
    static GlobalScope scope;

    @BeforeAll
    static void beforeAll() {
        ToolLanguage language = new ToolLanguage();
        ToolModelLoader loader = new ToolModelLoader(language);
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/symboltable").toAbsolutePath());
        ResolvingConfiguration configuration = new ResolvingConfiguration();
        ModelingLanguageFamily family = new ModelingLanguageFamily();

        family.addModelingLanguage(new ToolLanguage());
        family.addModelingLanguage(new EnvironmentLanguage());

        scope = new GlobalScope(modelPath, family);

        // Log.initDEBUG();
        Log.enableFailQuick(false);
        configuration.addDefaultFilters(language.getResolvingFilters());
        loader.loadModelsIntoScope("models.Tool", modelPath, scope, configuration);
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testSymbolTable() {
        ToolSymbol tool = scope.<ToolSymbol>resolve("models.Tool", ToolSymbol.KIND).get();
        List<AttributeSymbol> toolAttributes = tool.getAttributeSymbols();
        List<ResourceSymbol> toolResources = tool.getResourceSymbols();
        List<AttributeSymbol> resourceAttributes = toolResources.get(0).getAttributeSymbols();
        DockerfileSymbol dockerfile = toolAttributes.get(3).asEnvironmentAttribute().get().getEnvironmentSymbol().get();

        assertEquals(4, toolAttributes.size(), "There should be exactly three attributes in Tool.");
        assertEquals(1, toolResources.size(), "There should be exactly one resources in Tool.");
        assertTrue(dockerfile.isComponent(), "The referenced Dockerfile should be a component");
        assertEquals(2, resourceAttributes.size(), "There should be exactly two attributes in someResource.");
    }
}
