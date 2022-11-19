package de.monticore.lang.monticar.cnnarch.generator.training;

import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.generator.FreeMarkerTemplate;
import de.monticore.lang.monticar.cnnarch.generator.util.SchemaUtil;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import schemalang._ast.ASTComplexPropertyDefinition;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._ast.ASTTypedDeclaration;
import schemalang._symboltable.SchemaDefinitionSymbol;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class Schema2APITest {


    private GeneratorEngine generatorEngine;

    @BeforeEach
    void setUp() {
        GeneratorSetup generatorSetup = new GeneratorSetup();
        generatorSetup.setTracing(false);
        generatorSetup.setGlex(new GlobalExtensionManagement());
        generatorSetup.setOutputDirectory(new File("target/generated-sources/schemaAPI/"));
        this.generatorEngine = new GeneratorEngine(generatorSetup);
    }

    @Test
    void generateSchemaAPI() {
        final Schema2API schema2API = new Schema2API();
//        schema2API.generatePythonAPIs(schema2API.resolveSchemas());
    }

    @Test
    void parseSingleSchema() {
        List<ASTSchemaLangCompilationUnit> astSchemaDefinitions = new Schema2API().parseSchemas();
        assertNotNull(astSchemaDefinitions);
    }

    @Test
    void allSchemaTypes() {
        final Schema2API schema2API = new Schema2API();
        schema2API.setSchemasModelPath(new ModelPath(Paths.get("src/test/resources/schemas")));
        SchemaDefinitionSymbol supervised = schema2API.resolveSchema("Supervised");
        final ASTSchemaDefinition astSchemaDefinition = (ASTSchemaDefinition) supervised.getAstNode().get();
        final StringBuilder generationResult = generatorEngine.generate(FreeMarkerTemplate.SCHEMA_CLASS.getTemplateName(), astSchemaDefinition, astSchemaDefinition, null, astSchemaDefinition.getSchemaMemberList());
        System.out.print(generationResult);
//        IndentPrinter printer = new IndentPrinter();
//        new SchemaLang2OD(printer, new ReportingRepository(new ASTNodeIdentHelper())).handle(supervised.getSchemaDefinitionNode().get());
//        System.out.println(printer.getContent());
    }

    @Test
    void objectType2Python() throws IOException {
        final ASTSchemaDefinition schemaWithObjectTypeEntry = SchemaUtil.resolveASTSchemaDefinition("SchemaWithObjectTypeEntry", new ModelPath(Paths.get("src/test/resources/schemas")));
        final ASTTypedDeclaration optimizerMember = (ASTTypedDeclaration) schemaWithObjectTypeEntry.getSchemaMemberList().get(1);
        final ASTComplexPropertyDefinition optimizerTypeDefinition = SchemaUtil.getPropertyDefinitionForDeclaration(optimizerMember, schemaWithObjectTypeEntry);
        final StringBuilder generationResult = generatorEngine.generate(FreeMarkerTemplate.SCHEMA_API_OBJECTTYPE.getTemplateName(), optimizerMember, optimizerMember, optimizerTypeDefinition);
        final String expected = String.join("\n", Files.readAllLines(Paths.get("src/test/resources/schemaAPIs/SchemaWithObjectTypeEntry.py")));
        final String actual = String.join("\n", generationResult.toString());
        assertEquals(expected.trim(), actual.trim());
    }

    @Test
    void generateSupervisedConfigurationAPI() throws IOException {
        final ASTSchemaDefinition supervisedSchema = SchemaUtil.resolveASTSchemaDefinition("Supervised", new ModelPath(Paths.get("src/test/resources/schemas")));
        final Path generatedPath = Paths.get("target/generated-sources/schemaAPI/Supervised_Schema_API.py");
        new Schema2API().generatePythonAPIs(Collections.singletonList(supervisedSchema));
        final List<String> expectedFile = Files.readAllLines(Paths.get("src/test/resources/schemaAPIs/Supervised_Simplified.py"));
        final String expected = expectedFile.stream().map(String::trim)
                .filter(l -> !l.isEmpty())
                .collect(Collectors.joining("\n"));
        final List<String> actualFile = Files.readAllLines(generatedPath);
        final String actual = actualFile.stream().map(String::trim)
                .filter(l -> !l.isEmpty())
                .collect(Collectors.joining("\n"));
        assertEquals(expected, actual);
    }

    @Test
    void inheritanceOnRefModels() {
        final Schema2API schema2API = new Schema2API();
        schema2API.setSchemasModelPath(new ModelPath(Paths.get("src/test/resources/schemas")));
        SchemaDefinitionSymbol schema = schema2API.resolveSchema("SchemaWithReferenceModel");
        assertNotNull(schema.getAstNode().get());
        final ASTSchemaDefinition schemaDefinition = (ASTSchemaDefinition) schema.getAstNode().get();
    }

    @Test
    void referencemodelValidation() {
        Path src = Paths.get("src/test/resources/architectures");
    }
}