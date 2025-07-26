/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.symboltable;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentModelLoader;
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
        EnvironmentLanguage language = new EnvironmentLanguage();
        EnvironmentModelLoader loader = new EnvironmentModelLoader(language);
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/symboltable").toAbsolutePath());
        ResolvingConfiguration configuration = new ResolvingConfiguration();

        scope = new GlobalScope(modelPath, language);

        Log.enableFailQuick(false);
        configuration.addDefaultFilters(language.getResolvingFilters());
        loader.loadModelsIntoScope("root.Root", modelPath, scope, configuration);
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testSymbolTable() {
        DockerfileSymbol root = scope.<DockerfileSymbol>resolve("root.Root", DockerfileSymbol.KIND).get();
        List<DockerfileSymbol> components = root.getComponentSymbols();
        List<ASTInstruction> oneInstructions = components.get(0).getDockerfileNode().get().getInstructionList();
        List<ASTInstruction> twoInstructions = components.get(1).getDockerfileNode().get().getInstructionList();

        assertEquals(2, components.size(), "Root Dockerfile should import two Component Dockerfiles.");
        assertEquals(9, oneInstructions.size(), "ComponentOne should have exactly 9 instructions.");
        assertEquals(8, twoInstructions.size(), "ComponentTwo should have exactly 8 instructions.");
    }
}
