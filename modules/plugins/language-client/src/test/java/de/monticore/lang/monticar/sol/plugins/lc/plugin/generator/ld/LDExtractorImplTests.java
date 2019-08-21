/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageModelLoader;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateUndeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializer;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializerImpl;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.json.JSONArray;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;

public class LDExtractorImplTests {
    LDExtractorImpl extractor;
    ASTLanguageCompilationUnit emaComponent;
    ASTLanguageCompilationUnit emamComponent;

    @BeforeEach
    void before() {
        Injector injector = Guice.createInjector(new LanguageModule());
        OptionsSerializer serializer = injector.getInstance(OptionsSerializerImpl.class);
        LanguageModelLoader loader = injector.getInstance(LanguageModelLoader.class);
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources"));
        Collection<ASTLanguageCompilationUnit> emamNodes =
                loader.loadModelsIntoScope("LDExtractorImpl.EmbeddedMontiArcMath", modelPath);
        Collection<ASTLanguageCompilationUnit> emaNodes =
                loader.loadModelsIntoScope("LDExtractorImpl.EmbeddedMontiArc", modelPath);

        Log.enableFailQuick(false);
        Log.getFindings().clear();

        emamComponent = emamNodes.iterator().next();
        emaComponent = emaNodes.iterator().next();
        extractor = new LDExtractorImpl(serializer);
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
        List<TemplateUndeclarationSymbol> undeclarations = extractor.getTemplateUndeclarations(emamComponent);

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

    @Test
    void testGetElements() throws IOException {
        List<TemplateDeclarationSymbol> templates = extractor.getTemplateDeclarations(emaComponent);
        File elementsFile = Paths.get("src/test/resources/LDExtractorImpl/elements.json").toFile();
        String elementsContent = FileUtils.readFileToString(elementsFile, "UTF-8");
        JSONArray elements = new JSONArray(elementsContent);

        assertIterableEquals(
                elements.toList(),
                extractor.getElements(templates.get(0)).toList(),
                "Elements do not match."
        );
    }
}
