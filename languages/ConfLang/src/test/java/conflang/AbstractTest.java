/* (c) https://github.com/MontiCore/monticore */
package conflang;

import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfiguration;
import conflang._parser.ConfLangParser;
import conflang._symboltable.*;
import de.monticore.io.paths.ModelPath;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

import static org.junit.Assert.*;


/**
 * Provides some helpers for tests.
 */
public abstract class AbstractTest {

    /**
     * Parses a model and ensures that the root node is present.
     *
     * @param modelFile the full file name of the model.
     * @return the root of the parsed model.
     */
    protected static ASTConfLangCompilationUnit parse(String modelFile) {
        Path model = Paths.get(modelFile);
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTConfLangCompilationUnit> configuration;
        try {
            configuration = parser.parse(model.toString());
            assertFalse(parser.hasErrors());
            assertTrue(configuration.isPresent());
            return configuration.get();
        } catch (Exception e) {
            throw new RuntimeException(String.format("There was an exception when parsing the model '%s': %s", modelFile, e.getMessage()));
        }
    }

    protected ConfLangArtifactScope createSymbolTable(ASTConfLangCompilationUnit schemaLangCompilationUnit, String path) {
        final ModelPath modelPath = new ModelPath(Paths.get(path));
        final ConfLangLanguage schemaLangLanguage = new ConfLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(schemaLangLanguage.getResolvingFilters());

        ConfLangSymbolTableCreator symbolTableCreator = new ConfLangSymbolTableCreator(resolverConfiguration, new GlobalScope(modelPath, schemaLangLanguage));
        ConfLangArtifactScope schemaLangArtifactScope = (ConfLangArtifactScope) symbolTableCreator.createFromAST(schemaLangCompilationUnit);
        return schemaLangArtifactScope;
    }

    protected ConfigurationScope createConfigurationSymbolTable(ASTConfLangCompilationUnit schemaLangCompilationUnit, String path) {
        final ModelPath modelPath = new ModelPath(Paths.get(path));
        final ConfLangLanguage schemaLangLanguage = new ConfLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(schemaLangLanguage.getResolvingFilters());

        ConfLangSymbolTableCreator symbolTableCreator = new ConfLangSymbolTableCreator(resolverConfiguration, new GlobalScope(modelPath, schemaLangLanguage));
        ConfigurationScope schemaLangArtifactScope = (ConfigurationScope) symbolTableCreator.createFromAST(schemaLangCompilationUnit.getConfiguration());
        return schemaLangArtifactScope;
    }

    /* ============================================= HELPER METHODS ============================================= */

    protected ConfigurationSymbol parseAndGetConfLangConfigurationSymbol(String model, String modelPath) {
        ASTConfLangCompilationUnit confLangCompilationUnit = parse(model);
        createSymbolTable(confLangCompilationUnit, modelPath);
        ASTConfiguration confLangConfiguration = confLangCompilationUnit.getConfiguration();
        return confLangConfiguration.getConfigurationSymbol();
    }
}