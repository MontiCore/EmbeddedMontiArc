package de.monticore.lang.monticar.cnnarch.generator.training;

import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.generator.FreeMarkerTemplate;
import de.monticore.lang.monticar.cnnarch.generator.util.SchemaUtil;
import schemalang._ast.ASTSchemaDefinition;

import java.io.File;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Schema2API {

    private File outputDirectory = new File("target/generated-sources/schemaAPI");

    public void setOutputDirectory(File outputDirectory) {
        this.outputDirectory = outputDirectory;
    }

    public static void main(String[] args) {
        final String pathToSchemas = args[0];
        final List<String> schemaModels = new LinkedList<>(Arrays.asList(args).subList(1, args.length));
        final ModelPath modelPath = new ModelPath(Paths.get(pathToSchemas));
        final List<ASTSchemaDefinition> schemaDefinitions = new LinkedList<>();
        for (String schemaModel : schemaModels
        ) {
            schemaDefinitions.add(SchemaUtil.resolveASTSchemaDefinition(schemaModel, modelPath));
        }
        new Schema2API().generatePythonAPIs(schemaDefinitions);
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

    private void generateForSchema(final GeneratorEngine generatorEngine, final ASTSchemaDefinition schemaDefinition) {
        final ASTSchemaDefinition astSuperSchemaDefinition = schemaDefinition.getSuperSchemaDefinitions().stream().findFirst().orElse(null);
        generatorEngine.generate(FreeMarkerTemplate.SCHEMA_CLASS.getTemplateName(), Paths.get(schemaDefinition.getName() + "_Schema_API.py"), schemaDefinition,
                schemaDefinition, astSuperSchemaDefinition, schemaDefinition.getSchemaMemberList());
    }

}
