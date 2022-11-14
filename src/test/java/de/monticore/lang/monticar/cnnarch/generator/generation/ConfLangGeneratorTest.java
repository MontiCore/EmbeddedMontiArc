package de.monticore.lang.monticar.cnnarch.generator.generation;

import conflang._ast.ASTConfLangCompilationUnit;
import conflang._parser.ConfLangParser;
import de.monticore.lang.monticar.cnnarch.generator.transformation.TemplateLinker;
import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;

class ConfLangGeneratorTest {

    @Test
    void generatePythonTrainingConfiguration() throws IOException {
        final ASTConfLangCompilationUnit astConfLangCompilationUnit = new ConfLangParser().parse("src/test/resources/configurations/LeNetNetwork.conf")
                .orElseThrow(IllegalStateException::new);
        new TemplateLinker().linkToTrainingConfigurationTemplates(astConfLangCompilationUnit);
        final File pathToGeneratedFile = new File("target/generated-sources/configurations/Training_Configuration_LeNetNetwork.py");
        final File pathToExpectedFile = new File("src/test/resources/configurations/Training_Configuration_LeNetNetwork.py");
        new ConfLangGenerator().generatePythonTrainingConfiguration(astConfLangCompilationUnit);
        Assertions.assertEquals(FileUtils.readFileToString(pathToExpectedFile, "UTF-8"), FileUtils.readFileToString(pathToGeneratedFile, "UTF-8"));
    }
}