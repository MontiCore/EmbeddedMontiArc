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

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertNotNull;

class Schema2APITest {


    private GeneratorEngine generatorEngine;

    @BeforeEach
    void setUp() {
        GeneratorSetup generatorSetup = new GeneratorSetup();
        generatorSetup.setTracing(false);
        generatorSetup.setGlex(new GlobalExtensionManagement());
        this.generatorEngine = new GeneratorEngine(generatorSetup);
    }

    @Test
    void generateSchemaAPI() {
        new Schema2API().generatePythonAPI();
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
    void objectType2Python() {
        final Schema2API schema2API = new Schema2API();
        final ASTSchemaDefinition schemaWithObjectTypeEntry = SchemaUtil.resolveASTSchemaDefinition("SchemaWithObjectTypeEntry", new ModelPath(Paths.get("src/test/resources/schemas")));
        final ASTTypedDeclaration optimizerMember = (ASTTypedDeclaration) schemaWithObjectTypeEntry.getSchemaMemberList().get(2);
        final ASTComplexPropertyDefinition optimizerTypeDefinition = SchemaUtil.getPropertyDefinitionForDeclaration(optimizerMember, schemaWithObjectTypeEntry);
        final StringBuilder generationResult = generatorEngine.generate(FreeMarkerTemplate.SCHEMA_API_OBJECTTYPE.getTemplateName(), optimizerMember, optimizerMember, optimizerTypeDefinition);
        System.out.print(generationResult);
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