/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import com.google.inject.Guice;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateDeclaration;
import de.monticore.lang.monticar.sol.grammars.language._parser.LanguageParser;
import de.monticore.lang.monticar.sol.grammars.options.OptionsModule;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializer;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializerImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.json.JSONArray;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static org.mockito.Mockito.mock;

public class LDExtractorImplTests {
    LDExtractorImpl extractor;
    ASTLanguageCompilationUnit ast;

    @BeforeEach
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void before() throws Exception {
        LanguageParser parser = new LanguageParser();
        NotificationService notifications = mock(NotificationService.class);
        OptionsSerializer serializer = Guice.createInjector(new OptionsModule()).getInstance(OptionsSerializerImpl.class);
        File model = Paths.get("src/test/resources/LDExtractorImpl/EmbeddedMontiArcMath.ld").toFile();

        Log.enableFailQuick(false);
        Log.getFindings().clear();

        ast = parser.parse(model.getPath()).get();
        extractor = new LDExtractorImpl(notifications, serializer);
    }

    @Test
    void testGetTemplates() {
        assertEquals(1, extractor.getTemplates(ast).size(), "There should be exactly one template.");
    }

    @Test
    void testGetIdentifier() {
        List<ASTTemplateDeclaration> templates = extractor.getTemplates(ast);
        ASTTemplateDeclaration template = templates.get(0);

        assertEquals("embeddedmontiarc.emaComponent", extractor.getIdentifier(template), "ID does not match.");
    }

    @Test
    void testGetLabel() {
        List<ASTTemplateDeclaration> templates = extractor.getTemplates(ast);
        ASTTemplateDeclaration template = templates.get(0);

        assertEquals("EmbeddedMontiArc Component", extractor.getLabel(template), "Label does not match.");
    }

    @Test
    void testGetPath() {
        List<ASTTemplateDeclaration> templates = extractor.getTemplates(ast);
        ASTTemplateDeclaration template = templates.get(0);

        assertEquals("EmbeddedMontiArc.njk", extractor.getPath(template), "Path does not match.");
    }

    @Test
    void testGetElements() throws IOException {
        List<ASTTemplateDeclaration> templates = extractor.getTemplates(ast);
        ASTTemplateDeclaration template = templates.get(0);
        File elementsFile = Paths.get("src/test/resources/LDExtractorImpl/elements.json").toFile();
        String elementsContent = FileUtils.readFileToString(elementsFile, "UTF-8");
        JSONArray elements = new JSONArray(elementsContent);

        assertIterableEquals(elements.toList(), extractor.getElements(template).toList(), "Elements do not match.");
    }
}
