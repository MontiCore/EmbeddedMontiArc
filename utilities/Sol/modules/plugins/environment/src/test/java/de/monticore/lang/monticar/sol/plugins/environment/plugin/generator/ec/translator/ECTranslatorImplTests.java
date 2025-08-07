/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.translator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstall;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._ast.EnvironmentMill;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;
import de.se_rwth.commons.SourcePosition;
import org.apache.maven.project.MavenProject;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ECTranslatorImplTests {
    @Mock NotificationService notifications;
    @Mock EnvironmentGenerateConfiguration configuration;

    @InjectMocks
    ECTranslatorImpl translator;

    @Test
    void testTranslate() {
        List<List<ASTInstruction>> partitions = new ArrayList<>();
        List<ASTInstruction> partition = new ArrayList<>();
        ASTInstall instruction = EnvironmentMill.installBuilder().build();
        GeneratorEngine engine = mock(GeneratorEngine.class);
        StringBuilder result = new StringBuilder("Hello World!");
        String expected = String.format("%s\n", result.toString());

        partitions.add(partition);
        partition.add(instruction);
        when(engine.generateNoA("templates/INSTALL.ftl", partition)).thenReturn(result);

        assertEquals(expected, translator.translate(partitions, engine), "Expected Output differs from Actual Output.");
    }

    @Test
    void testPrintFilename() {
        File rootDirectory = new File("src/test/resources/DDFTranslatorImpl");
        File model = new File(rootDirectory, "Model.ddf");
        SourcePosition sourcePosition = SourcePosition.getDefaultSourcePosition();
        MavenProject mavenProject = mock(MavenProject.class);

        when(configuration.getMavenProject()).thenReturn(mavenProject);
        when(mavenProject.getBasedir()).thenReturn(rootDirectory);

        sourcePosition.setFileName(model.getPath());
        translator.printFilename(sourcePosition);

        assertNotEquals("", translator.printer.getContent(), "Printed Filename should not be empty.");

        translator.printer.clearBuffer();
        translator.printFilename(sourcePosition);

        assertEquals("", translator.printer.getContent(), "Filename should not have been printed.");
    }
}
