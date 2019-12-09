/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.*;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionLanguage;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class SymbolTableTests {
    static GlobalScope scope;

    @BeforeAll
    static void beforeAll() {
        LanguageLanguage language = new LanguageLanguage();
        LanguageModelLoader loader = new LanguageModelLoader(language);
        ModelingLanguageFamily family = new ModelingLanguageFamily();
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/symboltable").toAbsolutePath());
        ResolvingConfiguration configuration = new ResolvingConfiguration();

        family.addModelingLanguage(language);
        family.addModelingLanguage(new OptionLanguage());

        scope = new GlobalScope(modelPath, family);

        Log.enableFailQuick(false);
        configuration.addDefaultFilters(family.getAllResolvers());
        loader.loadModelsIntoScope("child.EmbeddedMontiArcDL", modelPath, scope, configuration);
    }

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testSymbolTable() {
        LanguageSymbol symbol = scope.<LanguageSymbol>resolve("child.EmbeddedMontiArcDL", LanguageSymbol.KIND).get();
        List<TemplateDeclarationSymbol> localDeclarations = symbol.getLocalDeclarationSymbols();
        List<TemplateExclusionSymbol> localUndeclarations = symbol.getLocalUndeclarationSymbols();
        List<OptionSymbol> options = localDeclarations.get(0).getOptionSymbols();
        Optional<TemplateDeclarationSymbol> declaration = localUndeclarations.get(0).getMatchingDeclarationSymbol();

        assertEquals(2, symbol.getParentSymbols().size(), "There should be exactly 2 parent language symbols.");
        assertEquals(1, localDeclarations.size(), "There should be exactly 1 declaration symbols.");
        assertEquals(3, options.size(), "There should be exactly 3 option symbols.");
        assertEquals(3, options.get(2).getOptions().size(), "There should be exactly 3 sub-options.");
        assertTrue(declaration.isPresent(), "There should be exactly one undeclaration with matching declaration.");
    }
}
