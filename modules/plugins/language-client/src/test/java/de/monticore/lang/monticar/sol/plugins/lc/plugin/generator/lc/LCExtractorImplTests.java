/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageModelLoader;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateExclusionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option.serializer.OptionSerializer;
import de.se_rwth.commons.logging.Log;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class LCExtractorImplTests {
    LCExtractorImpl extractor;
    ASTLanguageCompilationUnit emaComponent;
    ASTLanguageCompilationUnit emamComponent;
    Gson gson;

    @BeforeEach
    void before() {
        Injector injector = Guice.createInjector(new LanguageModule());
        GsonBuilder builder = new GsonBuilder();

        LanguageModelLoader loader = injector.getInstance(LanguageModelLoader.class);
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources"));
        Collection<ASTLanguageCompilationUnit> emamNodes =
                loader.loadModelsIntoScope("LDExtractorImpl.EmbeddedMontiArcMath", modelPath);
        Collection<ASTLanguageCompilationUnit> emaNodes =
                loader.loadModelsIntoScope("LDExtractorImpl.EmbeddedMontiArc", modelPath);

        Log.enableFailQuick(false);
        Log.getFindings().clear();

        gson = builder.registerTypeAdapter(OptionSymbol.class, new OptionSerializer()).setPrettyPrinting().create();
        emamComponent = emamNodes.iterator().next();
        emaComponent = emaNodes.iterator().next();
        extractor = new LCExtractorImpl(gson);
    }

    @Test
    void testGetTemplateDeclarations() {
        assertEquals(1, extractor.getTemplateDeclarations(emaComponent).size(), "There should be exactly one template.");
    }

    @Test
    void testGetTemplateUndeclarations() {
        assertEquals(1, extractor.getTemplateUndeclarations(emamComponent).size(), "There should be exactly one undeclaration.");
    }

    @Test
    void testGetIdentifier() {
        List<TemplateDeclarationSymbol> declarations = extractor.getTemplateDeclarations(emaComponent);
        List<TemplateExclusionSymbol> undeclarations = extractor.getTemplateUndeclarations(emamComponent);

        assertEquals(
                "LDExtractorImpl.EmbeddedMontiArc.emaComponent",
                extractor.getIdentifier(declarations.get(0)),
                "ID does not match."
        );

        assertEquals(
                "LDExtractorImpl.EmbeddedMontiArc.emaComponent",
                extractor.getIdentifier(undeclarations.get(0)),
                "ID does not match."
        );
    }

    @Test
    void testGetLabel() {
        List<TemplateDeclarationSymbol> templates = extractor.getTemplateDeclarations(emaComponent);

        assertEquals(
                "EmbeddedMontiArc Component",
                extractor.getLabel(templates.get(0)),
                "Label does not match."
        );
    }

    @Test
    void testGetPath() {
        List<TemplateDeclarationSymbol> templates = extractor.getTemplateDeclarations(emaComponent);

        assertEquals(
                "EmbeddedMontiArc.njk",
                extractor.getPath(templates.get(0)),
                "Path does not match."
        );
    }
}
