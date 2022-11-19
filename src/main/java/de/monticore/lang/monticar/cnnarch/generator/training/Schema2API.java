package de.monticore.lang.monticar.cnnarch.generator.training;

import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.generator.FreeMarkerTemplate;
import de.monticore.lang.monticar.cnnarch.generator.util.SchemaUtil;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._parser.SchemaLangParser;
import schemalang._symboltable.SchemaDefinitionSymbol;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Schema2API {

    private File outputDirectory = new File("target/generated-sources/schemaAPI");
    private ModelPath schemasModelPath = new ModelPath(Paths.get("src/main/resources/schemas"));

    public void setOutputDirectory(File outputDirectory) {
        this.outputDirectory = outputDirectory;
    }

    public void setSchemasModelPath(ModelPath schemasModelPath) {
        this.schemasModelPath = schemasModelPath;
    }

    /***
     * assumption: symbol table has been built on AST
     * @param schemaModels
     */
    public void generatePythonAPIs(List<ASTSchemaDefinition> schemaModels) {
        GeneratorSetup generatorSetup = new GeneratorSetup();
        generatorSetup.setTracing(false);
        generatorSetup.setOutputDirectory(outputDirectory);
        generatorSetup.setGlex(new GlobalExtensionManagement());
        GeneratorEngine generatorEngine = new GeneratorEngine(generatorSetup);
        for (ASTSchemaDefinition schemaDefinitionSymbol : schemaModels) {
            generateForSchema(generatorEngine, schemaDefinitionSymbol);
        }
    }

    private  void generateForSchema(final GeneratorEngine generatorEngine, final ASTSchemaDefinition schemaDefinition) {
        final ASTSchemaDefinition astSchemaDefinition = schemaDefinition;
        final ASTSchemaDefinition astSuperSchemaDefinition = astSchemaDefinition.getSuperSchemaDefinitions().stream().findFirst().orElse(null);
        generatorEngine.generate(FreeMarkerTemplate.SCHEMA_CLASS.getTemplateName(), Paths.get(schemaDefinition.getName() + "_Schema_API.py"), astSchemaDefinition,
                astSchemaDefinition, astSuperSchemaDefinition, astSchemaDefinition.getSchemaMemberList());
    }


    protected List<SchemaDefinitionSymbol> resolveSchemas() {
        List<SchemaDefinitionSymbol> schemas = new ArrayList<>();
        schemas.add(resolveSchema("Supervised"));
        return schemas;

    }

    protected SchemaDefinitionSymbol resolveSchema(final String schemaName) {
        return SchemaUtil.resolveSchemaDefinition(schemaName, schemasModelPath);

    }

    protected List<ASTSchemaLangCompilationUnit> parseSchemas() {
        List<ASTSchemaLangCompilationUnit> schemas = new ArrayList<>();
        schemas.add(parseSchema("General.scm"));
        schemas.add(parseSchema("Supervised.scm"));
        return schemas;
    }

    private ASTSchemaLangCompilationUnit parseSchema(final String schemaName) {
        SchemaLangParser schemaLangParser = new SchemaLangParser();
        final Optional<ASTSchemaLangCompilationUnit> astSchemaLangCompilationUnit;
        try {
            astSchemaLangCompilationUnit = schemaLangParser.parse("src/main/resources/schemas/" + schemaName);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return astSchemaLangCompilationUnit.orElseThrow(IllegalStateException::new);
    }
}
