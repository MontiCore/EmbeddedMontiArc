package schemalang.generator;

import com.google.common.collect.Lists;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaLangCompilationUnit;

import java.io.File;
import java.nio.file.Paths;
import java.util.List;

public class GeneratorTest extends AbstractTest {

    @Test
    public void generateConfigurationArtifacts() {

        generateConstants();
        generateConfigurationDataFacade();
    }

    private void generateConstants() {

        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/parser/SupervisedLearning.scm");
        ASTSchemaDefinition schemaLangDefinition = schemaLangCompilationUnit.getSchemaDefinition();

        GeneratorSetup s = new GeneratorSetup();
        s.setOutputDirectory(new File("src/main/java/generated"));
        s.setCommentStart("/*-- ");
        s.setCommentEnd(" --*/");
        s.setTracing(true);

        List<File> additionalTemplatePaths = Lists.newArrayList(new File("src/test/resources/schemalang/generator"));
        s.setAdditionalTemplatePaths(additionalTemplatePaths);

        GeneratorEngine ge = new GeneratorEngine(s);
        ge.generate("Constants", Paths.get("Constants.java"), schemaLangDefinition);
    }

    private void generateConfigurationDataFacade() {

        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/parser/SupervisedLearning.scm");
        ASTSchemaDefinition schemaLangDefinition = schemaLangCompilationUnit.getSchemaDefinition();

        GeneratorSetup s = new GeneratorSetup();
        s.setOutputDirectory(new File("src/main/java/generated"));
        s.setCommentStart("/*-- ");
        s.setCommentEnd(" --*/");
        s.setTracing(true);

        List<File> additionalTemplatePaths = Lists.newArrayList(new File("src/test/resources/schemalang/generator"));
        s.setAdditionalTemplatePaths(additionalTemplatePaths);

        GeneratorEngine ge = new GeneratorEngine(s);
        ge.generate("SchemaLang", Paths.get(schemaLangDefinition.getName() + ".java"), schemaLangDefinition);
    }
}