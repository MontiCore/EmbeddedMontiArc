package schemalang;/* (c) https://github.com/MontiCore/monticore */

import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfiguration;
import conflang._parser.ConfLangParser;
import conflang._symboltable.ConfLangLanguage;
import conflang._symboltable.ConfLangSymbolTableCreator;
import conflang._symboltable.ConfigurationScope;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._parser.EmbeddedMontiArcParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcArtifactScope;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import org.antlr.v4.runtime.RecognitionException;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._parser.SchemaLangParser;
import schemalang._symboltable.SchemaDefinitionScope;
import schemalang._symboltable.SchemaLangArtifactScope;
import schemalang._symboltable.SchemaLangLanguage;
import schemalang._symboltable.SchemaLangSymbolTableCreator;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

import static org.junit.Assert.*;


/**
 * Provides some helpers for tests.
 */
public abstract class AbstractTest {

    protected ASTSchemaLangCompilationUnit parse(String modelFile) {
        Path model = Paths.get(modelFile);
        SchemaLangParser parser = new SchemaLangParser();
        Optional<ASTSchemaLangCompilationUnit> configuration;
        try {
            configuration = parser.parse(model.toString());
            assertFalse(parser.hasErrors());
            assertTrue(configuration.isPresent());
            return configuration.get();
        } catch (RecognitionException | IOException e) {
            fail("There was an exception when parsing the model " + modelFile + ": "
                    + e.getMessage());
        }
        return null;
    }

    /**
     * Parses a model and ensures that the root node is present.
     *
     * @param modelFile the full file name of the model.
     * @return the root of the parsed model.
     */
    protected ASTSchemaDefinition parseSchemaDefinition(String modelFile) {

        ASTSchemaLangCompilationUnit parsedModel = parse(modelFile);
        if (parsedModel == null) {
            return null;
        }
        return parsedModel.getSchemaDefinition();
    }

    /**
     * Parses a model and ensures that the root node is present.
     *
     * @param modelFile the full file name of the model.
     * @return the root of the parsed model.
     */
    protected ASTConfiguration parseConfiguration(String modelFile) {
        Path model = Paths.get(modelFile);
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTConfLangCompilationUnit> configuration;
        try {
            configuration = parser.parse(model.toString());
            assertFalse(parser.hasErrors());
            assertTrue(configuration.isPresent());
            return configuration.get().getConfiguration();
        } catch (RecognitionException | IOException e) {
            fail("There was an exception when parsing the model " + modelFile + ": "
                    + e.getMessage());
        }
        return null;
    }

    protected ASTComponent parseEMAComponent(String modelFile) {
        Path model = Paths.get(modelFile);
        EmbeddedMontiArcParser parser = new EmbeddedMontiArcParser();
        Optional<ASTEMACompilationUnit> emaCompilationUnitOpt;
        try {
            emaCompilationUnitOpt = parser.parse(model.toString());
            assertFalse(parser.hasErrors());
            assertTrue(emaCompilationUnitOpt.isPresent());
            return emaCompilationUnitOpt.get().getComponent();
        } catch (RecognitionException | IOException e) {
            fail("There was an exception when parsing the model " + modelFile + ": "
                    + e.getMessage());
        }
        return null;
    }

    public static GlobalScope createSymbolTable() {
        final SchemaLangLanguage schemaLangLanguage = new SchemaLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(schemaLangLanguage.getResolvingFilters());
        final ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang"));

        return new GlobalScope(modelPath, schemaLangLanguage, resolverConfiguration);
    }

    public static SchemaDefinitionScope createSymbolTable(ASTSchemaDefinition schemaLangDefinition, String path) {
        final ModelPath modelPath = new ModelPath(Paths.get(path));
        final SchemaLangLanguage schemaLangLanguage = new SchemaLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(schemaLangLanguage.getResolvingFilters());

        SchemaLangSymbolTableCreator symbolTableCreator = new SchemaLangSymbolTableCreator(resolverConfiguration, new GlobalScope(modelPath, schemaLangLanguage));
        return (SchemaDefinitionScope) symbolTableCreator.createFromAST(schemaLangDefinition);
    }

    public static SchemaLangArtifactScope createSymbolTable2(ASTSchemaLangCompilationUnit schemaLangCompilationUnit, ModelPath modelPath) {
        final SchemaLangLanguage schemaLangLanguage = new SchemaLangLanguage();
        final EmbeddedMontiArcLanguage embeddedMontiArcLanguage = new EmbeddedMontiArcLanguage();
        final ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(schemaLangLanguage);
        family.addModelingLanguage(embeddedMontiArcLanguage);

        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(schemaLangLanguage.getResolvingFilters());
        resolverConfiguration.addDefaultFilters(embeddedMontiArcLanguage.getResolvingFilters());

        SchemaLangSymbolTableCreator symbolTableCreator = new SchemaLangSymbolTableCreator(resolverConfiguration,
                new GlobalScope(modelPath, family));
        return (SchemaLangArtifactScope) symbolTableCreator.createFromAST(
                schemaLangCompilationUnit);
    }

    protected static ConfigurationScope createEMASymbolTable(ASTConfiguration confLangConfiguration) {
        final ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/cocos"));
        final ConfLangLanguage confLangLanguage = new ConfLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(confLangLanguage.getResolvingFilters());

        ConfLangSymbolTableCreator symbolTableCreator = new ConfLangSymbolTableCreator(resolverConfiguration, new GlobalScope(modelPath, confLangLanguage));
        return (ConfigurationScope) symbolTableCreator.createFromAST(confLangConfiguration);
    }

    protected static EmbeddedMontiArcArtifactScope createEMASymbolTable(ASTComponent component) {
        final ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/cocos"));
        final EmbeddedMontiArcLanguage embeddedMontiArcLanguage = new EmbeddedMontiArcLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(embeddedMontiArcLanguage.getResolvingFilters());

        EmbeddedMontiArcSymbolTableCreator symbolTableCreator = new EmbeddedMontiArcSymbolTableCreator(resolverConfiguration,
                new GlobalScope(modelPath, embeddedMontiArcLanguage));
        Scope fromAST = symbolTableCreator.createFromAST(component);
        return null;
    }
}