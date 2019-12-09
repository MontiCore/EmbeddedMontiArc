/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageLanguage;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactLanguage;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactModelLoader;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SymbolTableTests {
    static GlobalScope scope;

    @BeforeAll
    static void beforeAll() {
        ArtifactLanguage language = new ArtifactLanguage();
        ArtifactModelLoader loader = new ArtifactModelLoader(language);
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/symboltable").toAbsolutePath());
        ResolvingConfiguration configuration = new ResolvingConfiguration();
        ModelingLanguageFamily family = new ModelingLanguageFamily();

        family.addModelingLanguage(new ArtifactLanguage());
        family.addModelingLanguage(new LanguageLanguage());
        family.addModelingLanguage(new EnvironmentLanguage());

        scope = new GlobalScope(modelPath, family);

        // Log.initDEBUG();
        Log.enableFailQuick(false);
        configuration.addDefaultFilters(family.getAllResolvers());
        loader.loadModelsIntoScope("models.Tool", modelPath, scope, configuration);
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testSymbolTable() {
        ToolSymbol tool = scope.<ToolSymbol>resolve("models.Tool", ToolSymbol.KIND).get();
        List<ArtifactSymbol> artifacts = tool.getArtifactSymbols();

        assertEquals("TOOL", tool.getAlias().get(), "Alias does not match.");
        assertEquals("some/imaginary/path", tool.getPath().get(), "Path does not match.");
        assertEquals(CommonLiterals.CWD, tool.getOrigin().get(), "Origin does not match.");
        assertEquals("java -jar", tool.getPrefix().get(), "Prefix does not match.");
        assertEquals(1, artifacts.size(), "There should be exactly one artifact.");

        ArtifactSymbol artifact = artifacts.get(0);

        assertEquals("RESOURCE", artifact.getAlias().get(), "Alias does not match.");
        assertEquals("some/imaginary/path", artifact.getPath().get(), "Path does not match.");
        assertEquals(CommonLiterals.ROOT, artifact.getOrigin().get(), "Origin does not match.");
    }
}
